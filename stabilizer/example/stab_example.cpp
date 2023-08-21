#include "stabilizer/stabilizer.h"
#include <ar10/testdata.h>
#include <arcore/offlineshooter.h>
#include <arcore/imageframe.h>
#include <warping/warping.h>
#include <ar10/filesystem.h>

#include <opencv2/opencv.hpp>

using namespace ar;
using namespace cv;
using namespace std;

string get_season_folder(const string &season_pattern)
{
  auto tp = ar::TestdataPosition::fromPattern(season_pattern);
  return afs::splitLast(tp.fullPath()).first;
}

static Camera read_cam_calibs(const string &season_pattern, int icam)
{
  auto season_dir = get_season_folder(season_pattern);
  Camera cam;
  if (icam == 0) /// центральная или левая
  {
    if (
      cam.read(afs::join(season_dir, "calib/left.yml")) ||
      cam.read(afs::join(season_dir, "calib/leftImage.yml")) ||
      cam.read(afs::join(season_dir, "calib/cam_mono.yml")) ||
      cam.read(afs::join(season_dir, "calib/cam_plg_left.yml")) ||
      cam.read(afs::join(season_dir, "calib/central60.yml")) ||
      cam.read(afs::join(season_dir, "../calib/cam_mono.yml")) ||
      cam.read(afs::join(season_dir, "../calib/cam_plg_left.yml"))
      )
      return cam;
    else
      cout << "Left/mono camera calibs not found" << endl;
  }
  return cam;
}

// todo: сделать замер времени
int main()
{
 Stabilizer stab(smoothing_method::KALMAN);
  SeasonList inp_seasons = {
    SeasonEntry("tor.265.%03d.", 1, 10), // сильно дёргается камера
  };


  for(const auto& season: inp_seasons)
  {
    SeasonShooter ss;
    ForwardRectify rectifying; // ректификация кадра
    // чтение данных камеры
    Camera camera_ = read_cam_calibs(season.seasonPattern, 0);
    // инициализация интрисиков
    Camera full_cam_opt = camera_;
    // ректификакция внутренних параметров камеры в зависимости от внешних
    full_cam_opt = rectifying.fRectCamera(full_cam_opt);
    TestdataPosition tp = TestdataPosition::fromPattern(season.seasonPattern);
    ss.init(tp.season().as<int>(), tp.serial());
    {
      // инициализируем шутер по шаблону сезона
      ar::SmartPtr<ar::OfflineShooter> offline_shooter(new ar::OfflineShooter);
      offline_shooter->set_lwmode(false); // если false - считывание avi и наоборот для картинок (true)
      ss.ConnectShooter(offline_shooter);
    }

    if(!ss.open(season.startEpisode, season.finishEpisode)){
      cerr << "season_shooter -> bad opening"; return -1;
    }
    else cout << "season_shooter -> OK" << endl;
    cout << "Opening episode range " << season.seasonPattern
         << ": " << season.startEpisode << " - " << season.finishEpisode << endl;

    // подготовка шутера
    ar::SmartPtr<ar::AShot> shot;
    ar::ImageFrame* ifr_le = nullptr;
    ar::TMapTypeFrames shot_frames = {};
    while(!(shot = ss.shoot()).isEmpty())
    {
      shot_frames = shot->inputFrames;
      if(shot_frames.find("leftImage") != shot_frames.end())
      {
        auto frame_le = shot_frames["leftImage"];
        ifr_le = frame_le.dynamicCast<ar::ImageFrame>();
        Mat left_image = rectifying.processStatic(ifr_le->src1, full_cam_opt);

        if (left_image.empty()) {
          cerr << "camera-src -> EMPTY" << endl;
          return -1;
        }

        auto stab_image = stab.feed_image_and_try_get_stab_it(left_image, shot->grabMsec());
        if(!stab_image.empty()){
          imshow("stab_video", stab_image.stabImg.value());
        }
        waitKey(1);
			}
		}
	}
  return 0;
}
