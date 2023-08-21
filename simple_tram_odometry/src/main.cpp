#include "precompiled.h"
#include "kittiodometer.h"
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;
using namespace ar;
// TODO
// 1) сделать возможность ехать назад (хафф в две стороны)
// 2) поработать над скоростью
// 3) сделать парсинг аргументов
// 4) найти места, которые можно ускорить (расспараллелить при инициализации всего)

KittiOdometer::KittiOdometer(bool use_config)
{
	if(!use_config){
		setup.bools["only_play_stand"] = true;             // режим наблюдения за стендом
		setup.bools["use_eval_poses_cache"] = false;       // использование кэшированных поз
		setup.ints["findEssMat_tune_iter"] = 2;            // кол-во итерация для улучшения сходимости EssMat
		setup.doubles["findEssMat_pixtresh"] = 0.5;        // макс. расстояние между корр. точками
		setup.doubles["findEssMat_prob"] = 0.9998;         // вероятность, что корр. точка будет удачной
		setup.bools["findEssMat_only_high_sigma"] = false; // игнорирование областей со слабым сигналом
		setup.doubles["findEssMat_xmargin"] = 0.10;        // отступ по X от границ при сопоставление ключевых точек
		setup.doubles["findEssMat_prox"] = 0.75;           // отступ по Y
		setup.doubles["minstep"] = 0.10;                   // минимальный шаг для пары кадров
		setup.sizes["grid_size"] = Size(27, 13);           // сетка оптического потока
		setup.bools["use_optflow_cache"] = false;          // кэш оптического потока
		setup.bools["write_GT_as_XYcoord"] = false;        // отписывание VR траектории
		setup.bools["draw_XYcoord_prev_start"] = false;    // отрисовка траектории проезда, полученной на прошлом запуске
		setup.bools["write_statistics"] = false;           // отписывание статистика всего прогона
		setup.bools["localizer_use_height"] = false;       // использование трамвайного локалайзер с учётом высоты
		setup.bools["localizer_trk"] = false;              // использование TRK локалайзера
		setup.bools["agro_localizer"]	= true;              // агро-локалайзер (с Калманом)
		setup.bools["draw_heights"] = false;               // отрисовка высот с STRM, данных геодезистов, данных проезда (Emlid)
		setup.bools["is_tram_stand"] = false;              // работа со стендом трамвая
		setup.bools["draw_google_map"] = true;             // отрисовка гугл карты в виде подложки (под обозначенями столбов и прочее)
		setup.bools["draw_and_use_segmentation"] = false;  // отрисовка сегментации и её использованием для нахождения ошибки репроекции
		setup.bools["test_localizer"] = true;              // режим тестирование локалайзера (например, на невязку сегментацию столбов
																											 // и их реальное положение на GeoMap)
		setup.bools["track_poles_as_line_reg"]  = false;   // трекинг столбов через их кластеризацию и predict как Line Regression
		setup.bools["track_poles_as_kalman_filter"] =false;// трекинг стационарных столбов через фильтр Калмана (выдаёт x_gm, y_gm)
		setup.bools["write_poles"] = false;                // отписывание столбов из разных источников
		setup.ints["delay"] = 1;                           // стартуем с паузы (0), ожидаем пробела или клавиши; иначе 1
		setup.bools["iter_by_image"] = false;
	}else
	{
		// TODO: чтение конфиг-файла
	}
  init_run();
}

int main()
{
  try
  {
    KittiOdometer odometer(false);

    SeasonList inp_seasons = {
      //SeasonEntry("trm.168.%03d.", 5, 999),
      //SeasonEntry("soy.304.%03d.", 1, 999),
      //SeasonEntry("soy.305.%03d.", 1, 999),
      SeasonEntry("psk.105.%03d.", 1, 999),
    };

    for(auto& season: inp_seasons){
      SeasonShooter ss;
      // определим координату-положение в датасете
      TestdataPosition tp = TestdataPosition::fromPattern(season.seasonPattern);
      ss.init(tp.season().as<int>(), tp.serial());
      {
        // инициализируем шутер по шаблону сезона
        ar::SmartPtr<ar::OfflineShooter> offline_shooter(new ar::OfflineShooter);
        offline_shooter->set_lwmode(false); // если false - считывание avi и наоборот для картинок (true)
        ss.ConnectShooter(offline_shooter);
      }

      if(!ss.open(season.startEpisode, season.finishEpisode)) return -1;
      else cout << "season_shooter -> OK" << endl;

      cout << "Opening episode range " << season.seasonPattern
           << ": " << season.startEpisode << " - " << season.finishEpisode << endl;

      // запустим прогон
      int key = odometer.process_avi(ss, season.seasonPattern);
      if (key == 'q' || key == 'Q')
        break;
    }

		if(odometer.setup.bools["write_poles"])
		{
			// TODO: сделать нормальный фильтр для сегментации
			ofstream poles_segm_report("segment_poles.csv", std::ofstream::app);
			ofstream poles_geomap_report("geomap_poles.csv", std::ofstream::app);
			poles_segm_report << "x\ty\n";
			poles_geomap_report << "x\ty\n";
			for(const auto& pole: odometer.ksdata.poles_gm_coords)
			{
				 Point2d middle_value = accumulate(pole.begin(), pole.end(), Point2d(0.0, 0.0),
																					 [&pole](const Point2d& pt1, const Point2d& pt2){
							Point2d add_value(pt2.x / pole.size(), pt2.y / pole.size());
							return pt1 + add_value;	});
				 poles_segm_report << to_string(middle_value.x) << "\t" << to_string(middle_value.y) << "\n";
			}
			for(const auto& pole: odometer.ksdata.poles_geomap_gm_coords)
			{
				poles_geomap_report << to_string(pole.x) << "\t" << to_string(pole.y) << "\n";
			}
			poles_segm_report.close();
			poles_geomap_report.close();

		}
		if(odometer.setup.bools["write_statistics"]){
			// отпишем всю полученную статистику по прогону
			KittiSequenceReport total;
			for (auto& sn2rep : odometer.seq_num2report)
			{
				auto rep = sn2rep.second;
				total.step_num += rep.step_num;
				total.speed_err_sum += rep.speed_err_sum;
				total.rotat_err_sum += rep.rotat_err_sum;
				total.angle_dif_sum += rep.angle_dif_sum;
				total.angle_tot_err += rep.angle_tot_err;
				total.duration += rep.duration;
			}
			odometer.ofs_report << "\tConfig: Release; svnrevision " << odometer.revision << endl;
			odometer.ofs_report << "Run number= \t" << odometer.run_number; // << endl;
			odometer.ofs_report << "\t Run date= \t" << odometer.run_date; // << endl;
			odometer.ofs_report << "\t Run time= \t" << odometer.run_time; // << endl;
		}
	}catch (exception& e)
	{
		cout << e.what() << endl;
	}

  return 0;
}
