/*
 * vizset_collector.hpp
 *
 *  Created on: Jul 26, 2024
 *      Author: hankm
 */

#ifndef VIZSET_COLLECTOR_HPP_
#define VIZSET_COLLECTOR_HPP_

#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <boost/filesystem.hpp>

#include "map_server/image_loader.h"
#include "yaml-cpp/yaml.h"
#include "opencv2/opencv.hpp"
#include "costmap_2d/costmap_2d.h"
#include "nav_msgs/MapMetaData.h"

#define ANG_STEP (0.01745) // 2*PI / 360
#define FOV_RADIUS (40)
#define UNKNOWN (127)
#define OCCUPIED (255)
#define FREE (0)


//#define DEBUG_DRAW
using namespace std;
using namespace costmap_2d;
//using VizSet = **MapLocation;
//using VizSetSize = vector<int> ;

namespace set_collector
{


class VizSetCollector
{
public:
	VizSetCollector();
	VizSetCollector(const nav_msgs::OccupancyGrid& gmap, const int& num_dowsamples);
	~VizSetCollector();

	void initialize() ;
	void rayTraceLines() ;
	void clusterToThreeLabels( cv::Mat& uImage  );
	void computeVisibiliies( );

	static void on_mouse(int event, int x, int y, int flags, void* userdata);
	void startMouseEvent(  ) ;
	cv::Mat GetMapImg(){ return mcv_map_orig; }
	cv::Mat* GetMapImgPtr() { return &mcv_map_orig; }
	//std::vector<VizSet> GetVizSet() const {return  mve_FreeCells; } ;
	// GetVizSet() {return m_vizset; };
	//vector<int> GetVizSetSize() const {return m_vizsetsize; }

	MapLocation** GetVizSetPtr() {return mppst_vizset; }
	int* GetVizSetSizePtr() {return mpn_vizsetsize; }

	int GetNumFullRays() {return mu_num_fullrays; }

private:

	cv::Mat mcv_map;
	cv::Mat mcv_map_orig ;
	int mn_numpyrdownsample ;
	int mn_scale ;
	uint8_t* mpcost_translation_table ;
	uint32_t mu_gmheight, mu_gmwidth ;

	// resized (downsampled) height and width
	uint32_t mu_cvheight, mu_cvwidth, mu_cvmapsize, mu_num_fullrays ;
	double mf_resolution, mf_ox, mf_oy;

	//std::vector<VizSet> mve_FreeCells ;
	costmap_2d::Costmap2D mo_costmap;

	//VizSet m_vizset ;
	//vector<int> m_vizsetsize ;
	int* mpn_vizsetsize ;
	MapLocation** mppst_vizset ;
};



}



#endif /* VIZSET_COLLECTOR_HPP_ */
