// этот файл в utf-8
#pragma hdrstop
#include "precompiled.h"
//#include <bits/stdc++.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/datasets/util.hpp>
#include <opencv2/ximgproc.hpp>

#include<tuple>

#include "kittiodometer.h"
#include "setup_optflow.h"

using namespace cv::ximgproc;
using namespace cv;
using namespace std;

inline cv::Vec3f homo(cv::Vec2f& p) { return Vec3f(p[0], p[1], 1); }

void drawStraightLine(cv::Mat img, cv::Point2f p1, cv::Point2f p2, cv::Scalar color)
{
   Point2f p, q;
   // Check if the line is a vertical line because vertical lines don't have slope
   if (p1.x != p2.x)
   {
           p.x = 0;
           q.x = img.cols;
           // Slope equation (y1 - y2) / (x1 - x2)
           float m = (p1.y - p2.y) / (p1.x - p2.x);
           // Line equation:  y = mx + b
           float b = p1.y - (m * p1.x);
           p.y = m * p.x + b;
           q.y = m * q.x + b;
   }
   else
   {
           p.x = q.x = p2.x;
           p.y = 0;
           q.y = img.rows;
   }

   cv::arrowedLine(img, p, q, color, 1);
}

void GammaCorrection(Mat& src, Mat& dst, float fGamma)
{
    unsigned char lut[256];
    for (int i = 0; i < 256; i++){
        lut[i] = saturate_cast<uchar>(pow((float)(i / 255.0), fGamma) * 255.0f);
    }
    dst = src.clone();
    const int channels = dst.channels();
    switch (channels){
    case 1:
    {
        MatIterator_<uchar> it, end;
        for (it = dst.begin<uchar>(), end = dst.end<uchar>(); it != end; it++)
            *it = lut[(*it)];
        break;
    }
    case 3:
    {
        MatIterator_<Vec3b> it, end;
        for (it = dst.begin<Vec3b>(), end = dst.end<Vec3b>(); it != end; it++)
        {
            (*it)[0] = lut[((*it)[0])];
            (*it)[1] = lut[((*it)[1])];
            (*it)[2] = lut[((*it)[2])];
        }
        break;
    }
    }
}

bool operator < (const cv::Vec2f u, const cv::Vec2f v)
{
  return u[0] < v[0] || (u[0] == v[0] && u[1] < v[1]);
}

//template< class T > 
void sort_two_vectors( 
  std::vector<Vec2f>& pps, 
  std::vector<Vec2f>& qqs,
  std::vector<double>& penalty 
)
{
  std::vector< pair< double, int > > wrk;
  for (int i = 0; i < pps.size(); i++)
    wrk.push_back(make_pair(penalty[i], i ) );
  sort(wrk.begin(), wrk.end());
  std::vector<Vec2f> ps; // (wrk.size(), Vec2f());
  std::vector<Vec2f> qs; // (wrk.size(), Vec2f());
  for (int i = 0; i < wrk.size(); i++)
  {
    penalty[i] = wrk[i].first;
    ps.push_back( pps[ wrk[i].second ] );
    qs.push_back( qqs[ wrk[i].second ] );
  }
  pps = ps;
  qqs = qs;
}

cv::Matx33d tuneEssentialMat(
  const cv::Matx33d& E, // essmat полученная из findEssentialMat после ранзака или лмедс
  std::vector<Vec2f>& hps, // уже переведенные в однородные координаты p- точки на которых строилась матрица в уравнении q E p = 0
  std::vector<Vec2f>& hqs, // уже переведенные в однородные координаты q- точки на которых строилась матрица в уравнении q E p = 0
  int iter,
  int max_iter
  )
{
  Matx33d E_tuned = E;
  // ... tuning ...
  vector< Vec2f > hhps, hhqs;
  vector< double > penalty;
  for (int i = 0; i < hps.size(); i++)
  {
    Vec2f& p = hps[i]; Vec2f& q = hqs[i];
    double err1 = homo(q).ddot(E * homo(p));
    penalty.push_back(abs(err1));
  }

  sort_two_vectors(hps, hqs, penalty);
  int max_points[] = { 15, 15, 25, 35, 50, 60 }; // ...
  int num_points = max( int(hps.size()) / 10, iter < 5 ? max_points[iter] : 60 );
  for (int i = 0; i < hps.size(); i++)
  {
    if (i > num_points)
      break;
    hhps.push_back(hps[i]);
    hhqs.push_back(hqs[i]);
  }

  num_points = hhps.size();

  Mat1d A(Size(9, num_points*3), 0.);
  for (int i = 0; i < hhps.size(); i++)
  {
    Vec2f& p = hhps[i]; Vec2f& q = hhqs[i];
    float px = p[0], py = p[1], qx = q[0], qy = q[1];
    A[i * 3 + 0][0] = px*qx; A[i * 3 + 0][1] = py*qx;  A[i * 3 + 0][2] = 1.*qx;
    A[i * 3 + 0][3] = px*qy; A[i * 3 + 0][4] = py*qy;  A[i * 3 + 0][5] = 1.*qy;
    A[i * 3 + 0][6] = px*1.; A[i * 3 + 0][7] = py*1.;  A[i * 3 + 0][8] = 1.;
  }

  Mat1d x;
  cv::SVD::solveZ(A, x);
  x /= x[0][8];
  E_tuned(0, 0) = x[0][0]; E_tuned(0, 1) = x[0][1];  E_tuned(0, 2) = x[0][2];
  E_tuned(1, 0) = x[0][3]; E_tuned(1, 1) = x[0][4];  E_tuned(1, 2) = x[0][5];
  E_tuned(2, 0) = x[0][6]; E_tuned(2, 1) = x[0][7];  E_tuned(2, 2) = x[0][8];

  return  E_tuned;
}

Point pt(Vec2f& v) { return Point(v[0], v[1]); }

void KittiOdometer::optflow_prepare(Snapshot& ss_pre, Snapshot& ss_cur)
{
  SFrame& pre = ss_pre.le;
  SFrame& cur = ss_cur.le;
  if (!setup.bools["use_optflow_cache"] || !read_optflow(cur.pre2cur_optflow, ss_pre.ipng, ss_cur.ipng, "le2le"))
  {
    cur.pre2cur_optflow = pre.pre2cur_optflow.clone();

    if(ksdata.optflower_DIS_motion == nullptr) // инициализация оптического потока
        ksdata.optflower_DIS_motion = setup_dis_optflow();
    {
      ksdata.optflower_DIS_motion->calc(cur.gre, pre.gre, cur.pre2cur_optflow);
    }

    if (setup.bools["use_optflow_cache"])
      write_optflow(cur.pre2cur_optflow, ss_pre.ipng, ss_cur.ipng, "le2le");

    cur.ipre = ss_pre.ipng; // запомним из какого фрейма был расчет потока в текущий
  }
}

Coor_points KittiOdometer::create_corr_points(const SFrame& pre, const SFrame& cur,
                                              const Mat1b& mask_low_sigma_grid,
                                              const cv::Point2d& PrP,
                                              const double focal)
{
  Size grid = setup.sizes["grid_size"];
  Mat2f pps(grid), qqs(grid);
  // переведём оптический поток в cv::Point-ы
  flow2gridpoints(cur.pre2cur_optflow, grid, pps, qqs);

  cv::Mat2f pps21 = pps.reshape(2, 1);
  cv::Mat2f qqs21 = qqs.reshape(2, 1);
  cv::Mat1b mask_low_sigma_grid_plain; // маска со слабым сигналом (засвет)
  if(mask_low_sigma_grid.cols > 1 && mask_low_sigma_grid.rows > 1)
    cv::Mat1b mask_low_sigma_grid_plain = mask_low_sigma_grid.reshape(1, 1);

  vector<Vec2f> _pps21, _qqs21;

  // отступы от точки (0, 0) и другие ограничения точек оптического потока
  double dx = cur.pre2cur_optflow.cols * setup.doubles["findEssMat_xmargin"];
  double dy = cur.pre2cur_optflow.cols / 20.;
  Rect2f roi(dx, dy, cur.pre2cur_optflow.cols - 2 * dx, cur.pre2cur_optflow.cols - 2 * dy);

  bool only_high_sigma = setup.bools["findEssMat_only_high_sigma"];

  // проверка точек на всевозможные ограничения по плоскости
  for (int x = 0; x < pps21.cols; x++)
  {
    if (!only_high_sigma || mask_low_sigma_grid_plain[0][x] > 0)
    {
      auto pp = pps21[0][x]; // точка из которой идет поток в qq
      auto qq = qqs21[0][x]; // точка в которую идет поток из pp
      if (qq[0] < roi.x || qq[0] > roi.x + roi.width  ||
          qq[1] < roi.y || qq[1] > roi.y + roi.height ||
          qq[1] < PrP.y)
        continue;
      if (pp[0] < roi.x || pp[0] > roi.x + roi.width || pp[1] < roi.y || pp[1] > roi.y + roi.height)
        continue;
      _pps21.push_back(pps21[0][x]);
      _qqs21.push_back(qqs21[0][x]);
    }
  }

  // P,Q - точки на земле
  vector< Vec2f > h_pps21, h_qqs21;
  for (int i = 0; i < _pps21.size(); i++)
  {
    auto& p = _pps21[i]; auto& q = _qqs21[i];
    Vec2d P((p[0] - PrP.x) / focal, (p[1] - PrP.y) / focal);
    Vec2d Q((q[0] - PrP.x) / focal, (q[1] - PrP.y) / focal);
    h_pps21.push_back(P); h_qqs21.push_back(Q);
  }
  return std::move(make_tuple(_pps21, _qqs21, h_pps21, h_qqs21));
}

void KittiOdometer::flow2gridpoints(const Mat2f flow, Size& grid, Mat2f& pps, Mat2f& qqs)
{
  Mat2f gridflow;
  resize(flow, gridflow, grid, INTER_AREA);
  double rx = double(gridflow.cols) / flow.cols;
  double ry = double(gridflow.rows) / flow.rows;
  gridflow.forEach(
    [&](Vec2f &pix, const int *pos)
  {
    const int& x = pos[1]; const int& y = pos[0];
    const Point2f flowatxy = gridflow[y][x];
    double flx = flowatxy.x;
    double fly = flowatxy.y;
    Vec2f pp((x + 0.5) / rx, (y + 0.5) / ry);
    pps[y][x] = pp;
    Vec2f qq = pp + Vec2f(flx, fly);
    qqs[y][x] = qq;
  });
}

inline bool point_inside_image( int xx, int yy, Mat mat)
{
  if (xx < 0 || yy < 0)
    return false;
  if (xx >= mat.cols || yy >= mat.rows)
    return false;

  return true;
}

cv::Mat1f solveFHT_collectHistogram( std::vector< cv::Point3d > abcs_x, bool horisontal)
{
  int NUM = 128;
  Size sz(2*NUM, 2*NUM);
  Point center(NUM, NUM);
  Mat1f AB(sz, 0);
  for (auto abc : abcs_x)
  {
    Point ab(center.x - (abc.x * NUM) / abc.z, center.y - (abc.y * NUM) / abc.z);
    if (ab.x >= 0 && ab.x < AB.cols)
      if (ab.y >= 0 && ab.y < AB.rows)
        AB(ab) += 10;
  }
  return AB;
}

void KittiOdometer::write_optflow(Mat2f& optflow, int prepng, int curpng, const char* prefix)
{
  string folder = dataset_folder + format("cache/optflow_seq%02d_%s/", ksdata.seq_num, prefix);
  cv::datasets::createDirectory(dataset_folder + "cache");
  cv::datasets::createDirectory(folder);
  string filename = folder + format("%05d-%05d.png", prepng, curpng);
  Mat1b m(optflow.rows, optflow.cols * 8, optflow.data);
  vector<int> parms;
  parms.push_back(IMWRITE_PNG_COMPRESSION);
  parms.push_back(0);
  imwrite(filename, m, parms);
}

bool KittiOdometer::read_optflow(Mat2f& optflow, int prepng, int curpng, const char* prefix)
{
  string folder = dataset_folder + format("cache/optflow_seq%02d_%s/", ksdata.seq_num, prefix);
  string filename = folder + format("%05d-%05d.png", prepng, curpng);
  Mat m2 = imread(filename, -1);
  if (m2.empty())
    return false;
  Mat flow2(m2.rows, m2.cols/8, CV_32FC2, m2.data);
  flow2.copyTo(optflow);
  return true;
}

Point2d estimate_FOE_fht(Mat2f forward_flow)
{
    float scale = 1. / 16;
    cv::Mat2f scaled_flow; resize(forward_flow, scaled_flow, Size(0, 0), scale, scale);
    double minddot = (0.05 * scaled_flow.cols)*(0.05 * scaled_flow.cols);

    double f = (scaled_flow.rows + scaled_flow.cols) / 2; // якобы фокусное расстояние
    vector< Point3d > abcs;   // полная коллекция нормалей
    static Mat1f hst(Size(256, 256), 0);
    int lost = 0;

    double sum_v = 0;
    double sum_u = 0;
    double sum_alpha = 0;

    for (int y = 0; y<scaled_flow.rows; y++ )
    for (int x = 0; x < scaled_flow.cols; x++)
    {
      Vec2f& uv = scaled_flow[y][x];
      if (uv.ddot(uv) < minddot)
        continue;
      double u = uv[0];
      double v = uv[1];
      Point3d p(x, y, f);
      Point3d q(u, v, f);
      Point3d abc = p.cross(q);
      // получили нормаль  к плоскости a*x+b*y+c*f = 0
      // теперь будем накапливать нормали и потом искать такой вектор foe что
      // foe.dot(abc_i) = 0 для всех или почти всех abc_i
      abc = abc / sqrt(abc.ddot(abc)); // отнормируем на 1
      if (abc.z < 0)
        abc *= -1;
      double alpha = x*v - y*u;
      int xx = cvRound(128 + v * 256 / alpha );
      int yy = cvRound(128 - u * 256 / alpha );
      if (point_inside_image(xx, yy, hst))
      {
        hst[yy][xx] += 10;
        sum_v += v;
        sum_u += u;
        sum_alpha += alpha;
      }
      else
        lost++;
      abcs.push_back(abc);
    }

    Mat1f AB = solveFHT_collectHistogram(abcs, true);   // abs(abc.x) > 2*abs(abc.y) -- премущественно горизонтальные линии
    cv::GaussianBlur(AB, AB, Size(3, 3), 0);

    Mat1f ABs;
    if (ABs.empty())
    ABs = AB.clone();
    else
    ABs += AB;

    Mat1f fht;
    FastHoughTransform(hst, fht, CV_32FC1, ARO_315_45, FHT_AVE, HDO_RAW);
    GaussianBlur(fht, fht, Size(3, 3), 0); // скромненько сгладим по окрестности
    Point ptmax, ptmin;
    double mi = 0; double ma = 0;
    minMaxLoc(fht, &mi, &ma, &ptmin, &ptmax);
    ptmax.x -= 128;
    ptmax.y -= 128;

    double kFx_div_Fy = ptmax.y / ptmax.x;

    double Fy = sum_alpha / (kFx_div_Fy*sum_v - sum_u);
    double Fx = kFx_div_Fy*Fy;

    Fx /= scale;
    Fy /= scale;
    return Point2d(Fx, Fy);
}

void KittiOdometer::find_essential(Snapshot& ss_pre, Snapshot& ss_cur)
{
  // todo: ускорить
  // todo: разобраться со сглаживаем PP
  // todo: сделать робастнее
  // todo: essential mat tuning, используя предыдущие кадры
  SFrame& pre = ss_pre.le;
  SFrame& cur = ss_cur.le; // левая камера (пока так до появления стерео)

  // настройка данных камеры
  double focal = cur.cam.projection(0, 0); // фокальная длина
  cv::Point2d PrP(cur.cam.projection(0, 2), cur.cam.projection(1, 2)); // Точка схода

  // подготовим оптический поток для создания корр. точек
  optflow_prepare(ss_pre, ss_cur);

  // автоматическое выделение и сглаживание точки схода на основе оптического потока
  /*Point2d foe_hough = ksdata.return_smoothedFOE(estimate_FOE_fht(cur.pre2cur_optflow, ss_cur.ipng, pre.gre, cur.gre));*/

  // подготовим маску, указывающие места с плохим сигналом
  Mat1b mask_low_sigma_grid;
  if (pre.mask_low_sigma.empty()){
    pre.mask_low_sigma = Mat::zeros(pre.gre.rows, pre.gre.rows, pre.gre.type());
    resize(pre.mask_low_sigma, mask_low_sigma_grid, setup.sizes["grid_size"], 0, 0, INTER_AREA);
  }

  // получим вектор корреспондирующих точек
  auto [_pps21, _qqs21, h_pps21, h_qqs21] = create_corr_points(pre, cur, mask_low_sigma_grid, PrP, focal);

  // E - основная матрица
  // R - матрица поворота
  // t - вектор сдвига
  Matx33d& E  = cur.E;
  Matx33d& R = cur.R;
  Vec3d& t = cur.t;

  Mat mask1; // аутлаеры
  {
    E = findEssentialMat(_pps21, _qqs21, focal, PrP, RANSAC,
      setup.doubles["findEssMat_prob"],
      setup.doubles["findEssMat_pixtresh"],
      mask1);

    int max_iter = setup.ints["findEssMat_tune_iter"];
    {
      for (int iter = 1; iter <= max_iter; iter++)
      {
        // улучшение EssentialMat, исходя из геометрии и физики
        E = tuneEssentialMat(E, h_pps21, h_qqs21, iter, max_iter);
      }
    }
  }


  Matx33f R1, R2;
  // получение единичного направления смещение и поворота
  decomposeEssentialMat(E, R1, R2, t);
  if (t[2] < 0)
    t = -t; // только вперед пока что. TODO: изменить

  if (R1(1,1) > R2(1,1))
    R = R1;
  else
    R = R2;


  Vec3f tr = R.t() * t; // вектор трансляции в координатах еще не повернутой камеры
  Vec3f htr(tr[0], tr[1], tr[2]); // xyz1
  Vec3f foe = cur.cam.projection * htr;
  if (foe[2] < 0)
    foe *= -1.;

  // NO ROTATION: вектор трансляции в координатах еще не повернутой камеры
  Vec3f ht(t[0], t[1], t[2]); // xyz1
  Vec3f foe_r = cur.cam.projection * ht;
  if (foe_r[2] < 0)
    foe_r *= -1.;

  Mat1b mask = mask1;

  // визуализация векторов оптического потока
  {
    Mat draw = ss_pre.le.bgr.clone();
    line(draw, Point(cur.cam.principalPoint().x, 0),
         Point(cur.cam.principalPoint().x, draw.rows), Scalar(0, 255, 255));
    line(draw, Point(0, cur.cam.principalPoint().y), Point(draw.cols, cur.cam.principalPoint().y), Scalar(0, 255, 255));

    for (int i = 0; i < mask.rows; i++)
    {
      Point pp = pt(_pps21[i]); // old image point
      Point qq = pt(_qqs21[i]); // new image point
      if(_pps21[i][1] > cur.cam.imageSize.height * 3 / 4)
      {
        arrowedLine(draw, pp, qq, Scalar(0, 255, 255), 1, 1, 0, 0.01);
      }
    }
    circle(draw, Point(PrP.x, PrP.y ), 6, Scalar(255, 0, 255), 5);
    imshow("Vis_optflow", draw);
  }
}
