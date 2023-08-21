// этот файл в utf-8
#include "precompiled.h" // #include "precompiled.h" обязателен для включения ПЕРВЫМ во все модули
#pragma hdrstop
#include "kittiodometer.h"

using namespace cv;
using namespace std;

double relerr(double delta_re, double delta_gt)
{
    return 2 * abs(delta_re - delta_gt) / abs(delta_re + delta_gt);
}

double radian(const Matx33f& Rot)
{
  Vec3f Rodr = {};  Rodrigues(Rot, Rodr);
  double deg = norm(Rodr);
  if (Rodr[1] < 0)
    deg = -deg;
  return deg;
}

double degree(const Matx33f& Rot)
{
  return radian(Rot)* 180. / CV_PI;
}

Vec3f rotationMatrixToEulerAngles_drawing(Mat &R)
{
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
    bool singular = sy < 1e-6;
    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return Vec3f(x, y, z);
}

double get_yaw(const Matx33f& Rot)
{
    Mat R(Rot);
    auto vec = rotationMatrixToEulerAngles_drawing(R);
    return vec[1] * 180. / CV_PI;
}
double get_roll(const Matx33f& Rot)
{
    Mat R(Rot);
    auto vec = rotationMatrixToEulerAngles_drawing(R);
    return vec[2] * 180. / CV_PI;
}
double get_pitch(const Matx33f& Rot)
{
    Mat R(Rot);
    auto vec = rotationMatrixToEulerAngles_drawing(R);
    return vec[0] * 180. / CV_PI;
}

inline float rotationError(Affine3d& pose_error) {
  float a = pose_error.matrix(0,0);
  float b = pose_error.matrix(1,1);
  float c = pose_error.matrix(2,2);
  float d = 0.5*(a + b + c - 1.0);
  return acos(max(min(d, 1.0f), -1.0f));
}

inline float rotationError(Matx33f R1, Matx33f R2) {
  Matx33f dR = R1.t()*R2;
  float a = dR(0, 0);
  float b = dR(1, 1);
  float c = dR(2, 2);
  float d = 0.5*(a + b + c - 1.0);
  float res = acos(max(min(d, 1.0f), -1.0f));
  return res;
}

inline float translationError(Affine3d &pose_error) {
  float dx = pose_error.matrix(0, 3);
  float dy = pose_error.matrix(1, 3);
  float dz = pose_error.matrix(2, 3);
  return sqrt(dx*dx + dy*dy + dz*dz);
}

vector<float> trajectoryDistances(vector<Affine3d> &poses) {
  vector<float> dist;
  dist.push_back(0);
  for (int32_t i = 1; i<poses.size(); i++)
  {
    Affine3d& P1 = poses[i - 1];
    Affine3d& P2 = poses[i];
    float delta = norm(P1.translation() - P2.translation());
    dist.push_back(dist[i - 1] + delta);
  }
  return dist;
}
void KittiSequenceData::register_sequence_window_error(int &ipng, const KittiErrors wnd, int iwnd )
{
  kitstat.t_err_sum[iwnd] += wnd.t_err / wnd.len; // суммарная ошибка трансляции по данному окну
  kitstat.r_err_sum[iwnd] += wnd.r_err / wnd.len; // суммарная ошибка поворота по данному окну
  kitstat.num_err[iwnd]++; // количество просуммированных ошибок по данному окну
  kitstat.t_err_sum[0] += wnd.t_err / wnd.len; // суммарная ошибка трансляции по всем окнам
  kitstat.r_err_sum[0] += wnd.r_err / wnd.len; // суммарная ошибка поворота по всем окнам
  kitstat.num_err[0]++; // количество просуммированных ошибок по всем окнам

  double pgt = path_gt[ipng] - path_gt[wnd.first_frame];
  double pre = path_re[ipng] - path_re[wnd.first_frame];
  double alpha = abs(pgt - pre) / sqrt(0.5*(pgt + pre));
  kitstat.alpha_sum[iwnd] += alpha; // суммарная альфа-ошибка по данному окну
  kitstat.alpha_sum[0] += alpha;    // суммарная альфа-ошибка по всем окнам
}

void KittiSequenceData::update_kitti_windows(int ipng)
{
    // на нулевом фрейме ошибок не может быть зарегистрировано
    if (ipng<= 0) return;

     // новая идеальная поза
    const Affine3d& gt_pose_cur = poses_gt[ipng];
    // новая расчитанная поза
    const Affine3d& re_pose_cur = poses_re[ipng];

    for (int iwnd = 1; iwnd <= 8; iwnd++) // окна [100, 200, ..., 800]
    {
        // как только окно "созрело", т.е. текущая gt_pose превысила длину траектории окна -- оно отписывается и его начало смещается
        KittiErrors& wnd = kitstat.windows[iwnd]; // окно расчета
        double len_required = iwnd * 100.; // требуемая длина цепочки [100, 200, ..., 800]
        while (1)
        {
            const Affine3d& gt_pose_start = poses_gt[wnd.first_frame]; // поза старта определённого окна GT
            const Affine3d& re_pose_start = poses_re[wnd.first_frame]; // поза старта полученной позы

            double dlen = path_gt[ipng] - path_gt[wnd.first_frame]; // длина траектории между началом окна и текущим кадром
            Vec3d gt_t = gt_pose_cur.translation() - gt_pose_start.translation(); // сдвиг GT позы
            Vec3d re_t = re_pose_cur.translation() - re_pose_start.translation(); // сдвиг полученной позы

            // находим сразу норму, так как СК(X, Y, Z) локалайзера могут не совпадать с X, Y, Z СК КИТТИ, а норма сдвига у них одна и та же
            auto diff_t = re_t - gt_t;

            Matx33f gt_R = gt_pose_cur.rotation() * gt_pose_start.rotation().t(); // поворот GT позы
            Matx33f re_R = re_pose_cur.rotation() * re_pose_start.rotation().t(); // поворот сдвиг полученной позы
            if (len_required >= dlen) break; // еще не перескочили предел окна. Ждём следующей итерации

            // заполним оценки ошибок для wnd
            wnd.len = len_required;
            wnd.gt_len = dlen;
            wnd.r_err = rotationError(gt_R, re_R) * 180. / CV_PI; // в градусах
            wnd.t_err = norm(diff_t);
            wnd.num_steps = ipng - wnd.first_frame; // зафиксируем сколько переходов в окне


            // отпишем окно, сдвинем на 1 и попробуем повторить
            register_sequence_window_error(ipng, wnd, iwnd);  // отписываем в vector общей статистики, в консоль, файл и т.п.
            wnd = KittiErrors(wnd.first_frame + 1);
        }
    }
}

void KittiSequenceData::update_discrepancy(int ipng)
{
    // на нулевом фрейме ошибок не может быть зарегистрировано
    if (ipng<= 0) return;

     // новая идеальная поза
    Affine3d gt_pose_cur = poses_gt[ipng];
    // новая расчитанная поза
    Affine3d re_pose_cur = poses_re[ipng];

    auto translation_GT = gt_pose_cur.translation();
    auto translation_RE = re_pose_cur.translation();

    double norm_translation_GT = norm(translation_GT);
    double norm_translation_RE = norm(translation_RE);

    kitstat.r_err_common += abs(norm_translation_GT - norm_translation_RE);
    kitstat.t_err_common += rotationError(gt_pose_cur.rotation(), re_pose_cur.rotation()) * 180. / CV_PI;
}

void KittiSequenceData::normalize_local_map()
{
    // выбираем самую близкую точку GT к границе local_map

    Point2d mi(1234567, 1234567);
    Point2d ma(-1234567, -1234567);
    for (auto& gtpose : poses_gt_drawing)
    {
        // выберем граничные точки графика, чобы можно было его (график) целиком вставить в смасштабированный квадрат
        auto gtt = gtpose.translation();
        mi.x = min(mi.x, gtt[0]); ma.x = max(mi.x, gtt[0]);
        mi.y = min(mi.y, -gtt[2]); ma.y = max(mi.y, -gtt[2]);
    }
    // начальная точка отсчёта
    auto st = poses_gt_drawing[0].translation();
    int point_startLine = 0; // индекс первой точки отрезка, по которому идёт вертикальное выравнивание GT
    int point_endLine = 0; // индекс второй точки отрезка, по которому идёт вертикальное выравнивание GT
    for (int jpng = 0; jpng < path_gt_drawing.size(); jpng++)
    {
        if(path_gt_drawing[jpng] < 2 && path_gt_drawing[jpng] > 0.1)
        {
            point_startLine = jpng;
        }
        if(path_gt_drawing[jpng] > 3)
        {
            point_endLine = jpng--;
            break;
        }
    }

    local_map_start = Point2d(st[0] + 100, -st[2]) - mi + Point2d(0, 100);

    auto start = Vec2d(poses_gt_drawing[point_startLine].translation()[1] + local_map_start.x,
                       -poses_gt_drawing[point_startLine].translation()[2] + local_map_start.y);

    auto end = Vec2d(poses_gt_drawing[point_endLine].translation()[1] + local_map_start.x,
                    -poses_gt_drawing[point_endLine].translation()[2] + local_map_start.y);

    if(end[1] > start[1]){
        strightAngle4GT += 180; // если по GT едем вниз, то меняем горизонтальную ориентацию
    }

    strightAngle4GT += atan2(end[0] - start[0], end[1] - start[1]) *  180.0 / CV_PI;

    for (size_t i = 0; i < poses_gt_drawing.size(); i++)
    {
        // отрисуем GT
        auto gtt = poses_gt_drawing[i].translation();
        Vec2d point_no_rot = { gtt[1] + local_map_start.x, -gtt[2] + local_map_start.y };
        Vec2d point_with_rot = gen_mat_rotation_map(strightAngle4GT) * point_no_rot;
        circle(local_map, Point((point_with_rot[0] / 20 + 300), (point_with_rot[1] / 20) + 500), 5, Scalar(220, 220, 220), 1);

        // синхронизируем нчальные точки отчёта для gt- и eval Pose
        if(i == 0){common_start_gt_eval =  Point(point_with_rot[0], point_with_rot[1]);}
    }

    // отрисовка метровых меток на собственной карте проезда
    // метки, что проехали 100 метров. Будем увеличивать каждые 100 метров
    int tt100 = 500;
    for (int jpng = 0; jpng < path_gt_drawing.size(); jpng++)
    {
        if (path_gt_drawing[jpng] > tt100)
        {
            auto gtt = poses_gt_drawing[jpng].translation();
            Vec2d point_no_rot = { gtt[1] + local_map_start.x, -gtt[2] + local_map_start.y };
            Vec2d point_with_rot = gen_mat_rotation_map(strightAngle4GT) * point_no_rot;

            point_with_rot[0] = (point_with_rot[0]) / 20 + 300;
            point_with_rot[1] = (point_with_rot[1] ) / 20 + 500;
            circle(local_map, Point(point_with_rot[0], point_with_rot[1]), 5, Scalar(150, 150, 150), 2);
            putText(local_map, format("%d", tt100), Point(point_with_rot[0], point_with_rot[1]) + Point(10, 10),
                    FONT_HERSHEY_COMPLEX_SMALL, .66, Scalar(128, 128, 128), 1, LINE_AA);
            tt100 += 500;
        }
    }
    path_re.push_back(0.);

    // вертикальные отметки
    for (int hh = -180; hh <= 180; hh += 10)
    {
        Point base = height_map_start + Point( 0, hh * height_pix_per_meter );
        line(height_map, base, base + Point(height_map.cols - base.x, 0), hh % 100 ? Scalar(220, 220, 220) : Scalar(128,128,128), hh ? 1 : 2 );
        putText(local_map, format("%d", (-hh) / 10 * 2), base + Point(-35, 10), FONT_HERSHEY_COMPLEX_SMALL, .66, Scalar(128, 128, 128), 1, LINE_AA);
    }

    // горизонтальные отметки
    int last_label = 0;
    for (int ix = 0;; ix++)
    {
        Point base1 = height_map_start + Point(ix * 10 * path_pix_per_meter, -400);
        Point base2 = height_map_start + Point(ix * 10 * path_pix_per_meter, +400);
        line(height_map, base1, base2, ix % 10 ? Scalar(220, 220, 220) : Scalar(128, 128, 128), ix ? 1 : 2 );
        if (ix % 2 == 0) {
            putText(local_map, format("%d", ix * 20), base2 + Point(-10, -10), FONT_HERSHEY_COMPLEX_SMALL, .66, Scalar(128, 128, 128), 1, LINE_AA);
            last_label = ix * 20;
        }
        if (base1.x > height_map.cols) break;
    }
    // проверим, что весь прогон влезит в график (920 метров), иначе продолжаем уже "с чистого листа"
    if(path_gt_drawing.back() > last_label)
    {
        isEnough = false;
    }

    // сделаем отдельное сохрание карты, чтобы характериситки не накладывались друг на друга
    local_map4reset = local_map.clone();
}

void KittiSequenceData::evaluate_poses_intermediate(int ipng, int iprev)
{
  auto gt_pose_cur = poses_gt[ipng]; // текущая интегральная поза (идеальная)
  auto re_pose_cur = poses_re[ipng]; // текущая интегральная поза (расчетная)

  // будем проверять покадровые метрики в режиме онлайн
  cout << "Frame=" << ipng << " => Translation (EVAL-vs-GT) diff = " << norm(re_pose_cur.translation()) - norm(gt_pose_cur.translation()) << endl;

  // отрисовка карты GT на первом кадре
  if (ipng < 1)
  {
      normalize_local_map();

  }else
  {
      auto gt_pose_pre = poses_gt[iprev]; // интегральная поза (идеальная) предыдущего кадра
      auto re_pose_pre = poses_re[iprev]; // интегральная поза (расчетная) предыдущего кадра

      // оценим мгновенный поворот и трансляцию на последнем кадре
      Matx33f gt_R_pre2cur = gt_pose_cur.rotation() * gt_pose_pre.rotation().t(); // эталонный поворот из pre в cur
      Matx33f re_R_pre2cur = re_pose_cur.rotation() * re_pose_pre.rotation().t(); // расчитанный поворот из pre в cur
      double delta_gt = norm(gt_pose_cur.translation() - gt_pose_pre.translation()); // норма истинной трансляции
      double delta_re = norm(re_pose_cur.translation() - re_pose_pre.translation()); // расчитанная норма трансляции на последнем кадре
      // добавим в общий путь полученное смещение
      path_re.push_back(path_re.back() + delta_re);

      double speed_err = relerr(delta_re, delta_gt); // контроль ошибок модуля скорости
      double rotat_err = rotationError(gt_R_pre2cur, re_R_pre2cur); // контроль ошибок поворота
      report.speed_err_sum += speed_err;
      report.rotat_err_sum += rotat_err;

      // расчитаем связанную систему координат (yaw, pitch, roll) и визуализируем их
      double gt_yaw = snapshots[0].yaw_gt;
      double re_yaw =  snapshots[0].yaw_eval;

      double gt_roll = snapshots[0].roll_gt;
      double re_roll = snapshots[0].roll_eval;

      double gt_pitch = snapshots[0].pitch_gt;
      double re_pitch = snapshots[0].pitch_eval;

      // отпшием ошибки смещения в файл
      double pgt = path_gt[ipng];
      double pre = path_re[ipng];
      double path_error = abs(pgt-pre) / sqrt(0.5*(pgt+pre));
      ofs_output << "\t path gt/re/err:\t " << pgt << '\t' << pre << '\t' << path_error << endl; // по скорости

      // обновим окно прохода по кадрам
      update_kitti_windows(ipng);

      // обновим ошибку невязки
      update_discrepancy(ipng);

      // график углов
      if (ipng > iprev)
      {
          draw_angle(gt_yaw, Scalar{0, 0, 255}, "gt_yaw", 35);
          draw_angle(re_yaw, Scalar{0, 150, 150}, "re_yaw", 50);
      }

      // отпишем мгновенную абсолютную ошибку по углу и число переходов step_num
      double angle_dif = abs(degree(gt_R_pre2cur * re_R_pre2cur.t()));
      report.angle_dif_sum += angle_dif;
      report.angle_tot_err = degree(re_pose_cur.rotation() * gt_pose_cur.rotation().t());
      report.step_num++;

      // отрисовка полученной траектории
      draw_trajectory(re_pose_cur.translation(), 180);

      // построим график скорости (в м/c)
      auto  dt_re = re_pose_cur.translation() - re_pose_pre.translation(); // вектор межкадрового смещения (расчет)
      auto  dt_gt = gt_pose_cur.translation() - gt_pose_pre.translation(); // вектор межкадрового смещения (идеал)
      if (ipng > iprev)
      {
          // выводим финальный local_map
          draw_speed(norm(dt_re), norm(dt_gt), ipng - iprev, norm(re_pose_cur.translation()) - norm(gt_pose_cur.translation()));
      }
  }
}

// цикл наполнения поз завершен. нужно сделать финальный расчет для сиквенса
void KittiOdometer::evaluate_poses_final(const string& output_folder, const Setup& setup)
{
    ofstream ofs_report_seq(output_folder + format("/report.run%04d.seq%02d.txt", run_number, ksdata.seq_num));

    KittiErrors tot;

    string info1 = format("Seq%02d\t Frames \t%03d\tRot_err \t%lf\t Trans_err \t%lf\t Alpha \t%lf\t",

                          ksdata.seq_num,
                          ksdata.poses_re.size(),
                          ksdata.kitstat.r_err_sum[0] / ksdata.kitstat.num_err[0],
                          ksdata.kitstat.t_err_sum[0] / ksdata.kitstat.num_err[0],
                          ksdata.kitstat.alpha_sum[0] / ksdata.kitstat.num_err[0]  );

    string info1_= format("Seq%02d   Frames   %03d  Rot_err   %lf   Trans_err   %lf   Alpha   %lf  ",
      ksdata.seq_num, ksdata.poses_re.size(), ksdata.kitstat.r_err_sum[0] / ksdata.kitstat.num_err[0],
      ksdata.kitstat.t_err_sum[0] / ksdata.kitstat.num_err[0], ksdata.kitstat.alpha_sum[0] / ksdata.kitstat.num_err[0]);

    int ilast = ksdata.poses_re.size() - 1;
    auto tre = ksdata.poses_re[ilast].translation();
    auto tgt = ksdata.poses_gt[ilast].translation();
    double dx = tre[0] - tgt[0];
    double dy = tre[1] - tgt[1]; // финальное отклонение по высоте
    double dz = tre[2] - tgt[2];
    double dxdz = norm(tgt) - norm(tre); // финальное отклонение на карте
    double dpath = ksdata.path_gt[ilast] - ksdata.path_re[ilast];

    string info2 = format("Final: \t err_path= \t%lf\t err_map= \t%lf\t", dpath, dxdz);
    string info2_= format("Fin0l:  err_path=   %lf   err_map=   %lf  ", dpath, dxdz);


    // сохранеям отдельно, чтобы текст не накладывался друг на друга
    Mat save_stat = ksdata.local_map.clone();

    putText(save_stat, info1_, Point(40 + 300, 40), FONT_HERSHEY_PLAIN, 1.5, Scalar(255, 0, 255), 1.5, LINE_AA);
    putText(save_stat, info2_, Point(40 + 300, 80), FONT_HERSHEY_PLAIN, 1.5, Scalar(255, 0, 255), 1.5, LINE_AA);
    imwrite(output_folder + format("/local_map.seq%02d.run%04d.png", ksdata.seq_num, run_number), save_stat);
    imshow("local_map (last seq -- final)", save_stat);

    ofs_report_seq << info1 << endl;
    ofs_report_seq << info2 << endl;
    ofs_report << info1 << "\t ";//endl;
    ofs_report << info2 << endl;
    ofs_report_seq << "--- by windows ---" << endl;
    for (int i = 1; i <= 8; i++)
    {
      KittiErrors& wnd = ksdata.kitstat.windows[i];
      //tot.r_err += wnd.r_err;
      //tot.t_err += wnd.t_err;
      string info_wnd = format("Seq%02d Len%03d Ro_err %lf Tr_err %lf Alpha %lf",
        ksdata.seq_num, i * 100,
        ksdata.kitstat.r_err_sum[i] / ksdata.kitstat.num_err[i],
        ksdata.kitstat.t_err_sum[i] / ksdata.kitstat.num_err[i],
        ksdata.kitstat.alpha_sum[i] / ksdata.kitstat.num_err[i]
        );
      ofs_report_seq << info_wnd << endl;
    }
    ofs_report_seq << "--- setup ---" << endl;
    print_user(ofs_report_seq);

    ofs_report_seq << setup;
}
