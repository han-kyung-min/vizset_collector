/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Brian Gerkey */

/* modified by han, kyungmin 2024.7.25 */

#define USAGE "\nUSAGE: map_server <map.yaml>\n" \
              "  map.yaml: map description file\n" \
              "OR: map_server <map> num_downsamples\n" \
              "  map: image file to load\n"\
              "  num_downsamples: downsampling counts"


//#include "ros/ros.h"
//#include "ros/console.h"

#include "vizset_collector.hpp"
#include "time.h"

using namespace set_collector;
using namespace std;

#ifdef HAVE_YAMLCPP_GT_0_5_0
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

const string window_name = "vizset";
cv::Mat dispimg, img_orig ;

class MapServer
{
  public:
    /** Trivial constructor */
    MapServer(const std::string& fname, double res = 0)
    {
      std::string mapfname = "";
      double origin[3];
      int negate;
      double occ_th, free_th;
      MapMode mode = TRINARY;
      std::string frame_id;
//      ros::NodeHandle private_nh("~");
//      private_nh.param("frame_id", frame_id, std::string("map"));
      deprecated = (res != 0);
      if (!deprecated) {
        //mapfname = fname + ".pgm";
        //std::ifstream fin((fname + ".yaml").c_str());
        std::ifstream fin(fname.c_str());
        if (fin.fail()) {
//          ROS_ERROR("Map_server could not open %s.", fname.c_str());
        	printf("Map_server could not open %s.", fname.c_str());
          exit(-1);
        }
#ifdef HAVE_YAMLCPP_GT_0_5_0
        // The document loading process changed in yaml-cpp 0.5.
        YAML::Node doc = YAML::Load(fin);
#else
        YAML::Parser parser(fin);
        YAML::Node doc;
        parser.GetNextDocument(doc);
#endif
        try {
          doc["resolution"] >> res;
        } catch (YAML::InvalidScalar &) {
          printf("The map does not contain a resolution tag or it is invalid.");
          exit(-1);
        }
        try {
          doc["negate"] >> negate;
        } catch (YAML::InvalidScalar &) {
        	printf("The map does not contain a negate tag or it is invalid.");
          exit(-1);
        }
        try {
          doc["occupied_thresh"] >> occ_th;
        } catch (YAML::InvalidScalar &) {
        	printf("The map does not contain an occupied_thresh tag or it is invalid.");
          exit(-1);
        }
        try {
          doc["free_thresh"] >> free_th;
        } catch (YAML::InvalidScalar &) {
        	printf("The map does not contain a free_thresh tag or it is invalid.");
          exit(-1);
        }
        try {
          std::string modeS = "";
          doc["mode"] >> modeS;

          if(modeS=="trinary")
            mode = TRINARY;
          else if(modeS=="scale")
            mode = SCALE;
          else if(modeS=="raw")
            mode = RAW;
          else{
        	  printf("Invalid mode tag \"%s\".", modeS.c_str());
            exit(-1);
          }
        } catch (YAML::Exception &) {
        	printf("The map does not contain a mode tag or it is invalid... assuming Trinary");
          mode = TRINARY;
        }
        try {
          doc["origin"][0] >> origin[0];
          doc["origin"][1] >> origin[1];
          doc["origin"][2] >> origin[2];
        } catch (YAML::InvalidScalar &) {
        	printf("The map does not contain an origin tag or it is invalid.");
          exit(-1);
        }
        try {
          doc["image"] >> mapfname;
          // TODO: make this path-handling more robust
          if(mapfname.size() == 0)
          {
        	  printf("The image tag cannot be an empty string.");
            exit(-1);
          }

          boost::filesystem::path mapfpath(mapfname);
          if (!mapfpath.is_absolute())
          {
            boost::filesystem::path dir(fname);
            dir = dir.parent_path();
            mapfpath = dir / mapfpath;
            mapfname = mapfpath.string();
          }
        } catch (YAML::InvalidScalar &) {
        	printf("The map does not contain an image tag or it is invalid.");
          exit(-1);
        }
      } else {
//        private_nh.param("negate", negate, 0);
//        private_nh.param("occupied_thresh", occ_th, 0.65);
//        private_nh.param("free_thresh", free_th, 0.196);
    	  negate = 0 ;
    	  occ_th = 0.65 ;
    	  free_th = 0.196 ;
        mapfname = fname;
        origin[0] = origin[1] = origin[2] = 0.0;
      }

      printf("Loading map from image \"%s\"", mapfname.c_str());
      try
      {
          map_server::loadMapFromFile(&map_resp_,mapfname.c_str(),res,negate,occ_th,free_th, origin, mode);
      }
      catch (std::runtime_error e)
      {
    	  printf("%s", e.what());
          exit(-1);
      }
      // To make sure get a consistent time in simulation
      //ros::Time::waitForValid();
      //map_resp_.map.info.map_load_time = ros::Time::now();
      map_resp_.map.header.frame_id = frame_id;
      //map_resp_.map.header.stamp = ros::Time::now();
      printf("Read a %d X %d map @ %.3lf m/cell",
               map_resp_.map.info.width,
               map_resp_.map.info.height,
               map_resp_.map.info.resolution);
      meta_data_message_ = map_resp_.map.info;
    }

    nav_msgs::OccupancyGrid GetMap( )
    {
    	return map_resp_.map ;
    }


  private:

    bool deprecated;

    nav_msgs::MapMetaData meta_data_message_;
    nav_msgs::GetMap::Response map_resp_;
};

//void on_mouse(int event, int x, int y, int flags, vector<VizSet> userdata)
void on_mouse(int event, int x, int y, int flags, void* userdata)
{
	//cout << "flags: " << flags << endl;
	//std::vector<VizSet>* pmydata = (std::vector<VizSet>*) userdata;
	//VizSet* pmydata = (VizSet*) userdata ;
	//VizSet* pstart_ptr = pmydata ;
//	MapLocation *ploc = (MapLocation*) userdata;

clock_t alloc_start = clock();
	VizSetCollector* ptr = (VizSetCollector*) userdata ;
	int num_fullrays = ptr->GetNumFullRays() ;

	MapLocation** ppvizset = ptr->GetVizSetPtr() ;
	VizSetSize vizsetsize = ptr->GetVizSetSize();

clock_t alloc_end = clock();
double alloc_time = double(alloc_end - alloc_start)/CLOCKS_PER_SEC;
printf("alloc time: %f \n", alloc_time);

    if ( event == cv::EVENT_LBUTTONDOWN )
    {
    	printf("tot num full rays %d \n", num_fullrays );
    	cv::cvtColor(img_orig, dispimg, cv::COLOR_GRAY2RGB);
        std::cout << "Left button of the mouse is clicked in on_mouse() func (" << x << ", " << y << ")" << std::endl;
        std::cout << ptr->GetMapImg().cols << " " << ptr->GetMapImg().rows << endl;
        int minerror = 10000000;

        int rcentidx = 0 ;
        clock_t start_search = clock() ;
        for( int sidx=0; sidx < num_fullrays; sidx++)
        {
        	uint32_t cx = ppvizset[sidx][0].x ;
        	uint32_t cy = ppvizset[sidx][0].y ;
printf("sidx: %d cxy: %d %d \n", sidx, cx, cy);
        	int error = sqrt( (cx - x)*(cx - x) + (cy - y)*(cy - y) ) ;
        	if(error < minerror)
        	{
        		minerror = error ;
        		//printf("%d %d\n", error, minerror);
        	}
			// debug
        	if( error < 4 )
        	{
        		rcentidx = sidx ;
        		break ;
        	}
        }

        clock_t end_search = clock();
        double search_time = double(end_search - start_search)/CLOCKS_PER_SEC;

        clock_t start_draw = clock() ;
		int num_vizset = vizsetsize[rcentidx];
		for( int midx=0; midx < num_vizset; midx++ )
		{
			cv::circle(dispimg, cv::Point(ppvizset[rcentidx][midx].x, ppvizset[rcentidx][midx].y), 2, cv::Scalar(0,255,255), 1, 8, 0) ;
		}

        clock_t end_draw = clock();
        double draw_time = double(end_draw - start_draw)/CLOCKS_PER_SEC;
        printf("search time %.3fs / draw time %.3fs \n", search_time, draw_time);
		cv::imshow(window_name, dispimg);
    }
}

int main(int argc, char **argv)
{
  if(argc != 3 && argc != 2)
  {
    printf("%s", USAGE);
    exit(-1);
  }

  std::string fname(argv[1]);
  int num_downsamples = (argc == 2) ? 0 : atof(argv[2]);

// load map from img

  nav_msgs::GetMap::Response map_resp_;
  int negate;
  double occ_th, free_th;
  MapMode mode = TRINARY;
  std::string mapfname = "";
  double origin[3];

  MapServer ms(fname) ;

  nav_msgs::OccupancyGrid gmap = ms.GetMap() ;

  VizSetCollector oVizSetCollector( gmap, num_downsamples );
  oVizSetCollector.initialize();
  oVizSetCollector.computeVisibiliies();

//  VizSet vizset = oVizSetCollector.GetVizSet() ;
//  for (int idx=0; idx< 11; idx++)
//	  printf("%d %d \n", vizset[idx][0].x, vizset[idx][0].y);

  img_orig = oVizSetCollector.GetMapImg().clone();
  cv::cvtColor(img_orig, dispimg, cv::COLOR_GRAY2RGB);
  cv::namedWindow(window_name);

  VizSetCollector* pVizSetCollector = &oVizSetCollector ;
  cv::setMouseCallback(window_name, on_mouse, pVizSetCollector ) ;

  imshow(window_name, dispimg);
  cv::waitKey(0);

  cv::destroyAllWindows();

  return 0;
}



