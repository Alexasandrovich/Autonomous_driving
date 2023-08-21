#include "get_data.h"
#include "vis_geo_spline.h"

int main(int argc, char** argv)
{
  try{
    Data_origination d;
    if(argc == 3) d = Data_origination(argv[1], argv[2], argv[3]);
    else std::cout << "\n!!! You also can set custom paths to spline, srtm (geomap), geodetic data !!!\n\n";

    // reading spline
    const auto& spline = d.read_spline();

    // getting data from geodetic measurements
    const auto& geodetic_data = d.read_geodetic_data(); // read .csv using sx::ifdelimstream

    /* ALTERNATIVE:
     * read .tsv using ObjIStream, but it depends correct format (TSV)
     * s.read_geodetic_data_as_smart();
     */

    // initing geomap and getting data using geodetic measurements
    d.get_data_from_geomap();

    // writing in tsv-file
    d.merge_data_and_write_tsv();

    // visual
    GeoSpline_display data_vis;
    if(spline.has_value() && geodetic_data.has_value())
    {
      data_vis.draw_lapped_splines(spline.value(), geodetic_data.value());
      data_vis.draw_normalized_heights(d.get_heights());
      data_vis.draw_diff_lapped_splines(d.get_dist2spline());
      data_vis.display_and_save(d.get_path_xy());
    }
    return 0;

  }catch(...)
  {
    ARLOG("geo_spline_vis", "bad init - check paths");
  }
  return 0;
}
