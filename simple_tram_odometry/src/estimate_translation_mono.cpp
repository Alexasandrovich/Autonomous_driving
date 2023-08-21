// этот файл в utf-8
#pragma hdrstop

#include "precompiled.h"
#include "estimate_translation.h"
#include <opencv2/ximgproc.hpp>
#include <warping/warping.h>
#include <cmath>

using namespace cv;
using namespace std;
using namespace cv::ximgproc;

double SQ_(double x) {
  return x*x;
}

void create_birdeye_place(const Size& imgsize,vector<cv::Point>& contour)
{
  contour.push_back(Point(imgsize.width * 0.30, imgsize.height *  0.7));
  contour.push_back(Point(imgsize.width * 0.65, imgsize.height * 0.7));
  contour.push_back(Point(imgsize.width * 0.9, imgsize.height));
  contour.push_back(Point(imgsize.width * 0.1, imgsize.height));
  contour.push_back(Point(imgsize.width * 0.30, imgsize.height * 0.7));
}

int estimate_translation_mono(Snapshot& ss_pre, Snapshot& ss_cur, SpeedMap& speedmap, Setup& setup)
{
  Mat2f ROI_flow = ss_cur.le.pre2cur_optflow.clone(); // оптический поток для зоны-бёрдай
  BirdeyeTransformer brd;
  brd.init(ss_pre.le.cam, Size(700, 1200));

  Size imgsize = ROI_flow.size();

  if (ROI_flow.empty())
    return 0;

  // инициализация хар-ик камеры
  double focal = ss_pre.le.cam.projection(0,0); // фокальная длина
  double height = ss_pre.le.cam.getExPose().getZ(); // высота камеры относительно плоскости дороги
  Point2f pp(ss_pre.le.cam.projection(0, 2), ss_pre.le.cam.projection(1, 2)); // Principal Point

  int cycle = speedmap.hist_orig.rows; // размер циклического буфера == 1200 секунд
  int row = ss_cur.ipng % cycle; // позиция кадра в буфере

  // гистограмма для расчета смещений (вертикаль - номер кадра, горизонталь - смещение в пикселях)
  Mat1f trans_cm = speedmap.hist_orig( Rect(0, row, speedmap.hist_orig.cols, 1));
  trans_cm = 0; // заполняем голоса смещений пикселей в ОДНОМ кадре

  int HOUGH_LEN = 8; // 8
  Mat1f hough_cm; // берём HOUGH_LEN кадров (== история) и ищем прямую, которая показывает вероятное смещение
  if (row > HOUGH_LEN)
  {
    // гистограмма для расчета хафом ПОЛУсантиметрах
    hough_cm = speedmap.hist_orig(Rect(0, row - HOUGH_LEN + 1, speedmap.hist_orig.cols, HOUGH_LEN));
  }

  // если зона засвеченна, то не смотрим ключевые точки, которые нашлись на ней
  if (ss_pre.le.mask_low_sigma.empty())
    ss_pre.le.mask_low_sigma = binarize_low_sigma_zones(ss_pre.le);

  int denom = ss_cur.ipng - ss_pre.ipng; // учтем число кадров

  // говорим, что интересны только те области бёрдая, которые не засвеченны
  bool only_high_sigma = setup.bools["estimate_translation_only_high_sigma"];

  // для проверки, что смещение корректное
  short invalid_displacement = 0x7fff;

  Mat1i dispmap(imgsize, invalid_displacement); // карта сдвигов (pix[x, y] = translation)

  int xgap = imgsize.width * 0.10; // отрежем шум по краям
  int ygap = imgsize.height * 0.05;

  vector<Point> contour;
  create_birdeye_place(imgsize, contour); // выделим бёрдай область

  int valid_point = 0; // кол-во точек, по которым идёт оценка смещения
  ROI_flow.forEach([&](Vec2f &uv, const int *pos)
  {
    // p_pre = [x - PP.x, y - PP.y, focal]
    // p_cur = [x - PP.x + flow.x, y - PP.y + flow.y, focal]
    const int x = pos[1]; const int y = pos[0];

    if (pointPolygonTest(contour, Point(x, y), false) < 0)
    { // берём точки только из бёрдай (гомография только для плоскости)
          return;
    }

    if (x < xgap || x + xgap >= imgsize.width || y < ygap || y + ygap >= imgsize.height)
      return;

    if (only_high_sigma && ss_pre.le.mask_low_sigma[y][x] == 0) // а если одна из точек-таки нулевая
      return;
    int xx = cvRound(x + uv[0]);
    int yy = cvRound(y + uv[1]);
    if (yy < 0 || yy >= imgsize.height || xx < 0 || xx >= imgsize.width)
        return;

    Vec3d p_pre(x - pp.x, y - pp.y, focal); // точка на старом кадре
    Vec3d p_cur(x + uv[0] - pp.x, y + uv[1] - pp.y, focal);// точка на текущем кадре

    // сперва поворачиваем камеру
    Vec3d p_pre_rot = ss_cur.le.R * p_pre;
    // восстанавливаем точки на земле,
    Vec3d P_pre_rot = p_pre_rot * (height / p_pre_rot[1]);
    // восстанавливаем вторую точку на земле (после трансляции, уже без поворота)
    Vec3d P_cur = p_cur * (height/p_cur[1]);
    // получиаем вектор трансляции
    const Vec3d trans = P_pre_rot - P_cur;

    // коэффициент нужен для масштаба полученного смещения (в пикселях) в сантиметры
    double half_cm = 200 * norm(trans);
    valid_point++;

    if (denom > 0) // число фреймов
    {
      int disp = cvRound(half_cm / denom);
      // (pix => translation)
      dispmap[y][x] = disp;
    }
  });


  for (int y = 0; y < imgsize.height; y++)
  {
      for (int x = 0; x < imgsize.width; x++)
      {
        int disp = dispmap[y][x];
        if (disp >=0 && disp < trans_cm.cols)
          // заполняем смещения для ТЕКУЩЕГО кадра
          trans_cm[0][disp]++;
      }
  }

  Point ptma = {};
  if (!hough_cm.empty())
  {
    Mat1f hough_cm2; resize(hough_cm, hough_cm2, Size(hough_cm.cols / 2, hough_cm.rows * 2),
                            0, 0, cv::INTER_AREA);
    Mat1f fht; // пространство Хафа для поиска прямой
    FastHoughTransform(hough_cm2, fht, CV_32FC1, ARO_315_135, FHT_AVE, HDO_DESKEW);
    GaussianBlur(fht, fht, Size(7, 7), 0); // сглаживание, чтобы убрать шум
    Point hp = get_max_pos(fht); // максимальная позиция точки в пространстве Хафа
    auto hl = HoughPoint2Line(hp, hough_cm2, ARO_315_135, HDO_DESKEW);
    if (hl[1] == hough_cm2.rows - 1)
      ptma.x = hl[0] * 2;
  }

  ss_cur.translation = ptma.x * 0.005 * denom;
  ss_cur.le.t = (ss_cur.le.t / norm(ss_cur.le.t)) * ss_cur.translation; // получаем ВЕКТОР смещения в полусантиметрах
  cout << "****** shot " << ss_pre.ipng << " to " << ss_cur.ipng << " ************ translation=" << ss_pre.translation << endl;
  return valid_point;
}
