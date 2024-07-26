/*
 * raytrace.cpp
 *
 *  Created on: Jul 25, 2024
 *      Author: hankm
 */

#include "vizset_collector.hpp"

namespace set_collector
{

VizSetCollector::VizSetCollector( const nav_msgs::OccupancyGrid& gmap, const int& num_dowsamples ):
mn_numpyrdownsample(num_dowsamples),
mpcost_translation_table(NULL)
{
	mpcost_translation_table = new uint8_t[101];

	// special values:
	mpcost_translation_table[0] = 0;  // NO obstacle
	mpcost_translation_table[99] = 253;  // INSCRIBED obstacle
	mpcost_translation_table[100] = 254;  // LETHAL obstacle
//  pcost_translation_table[-1] = 255;  // UNKNOWN

	// regular cost values scale the range 1 to 252 (inclusive) to fit
	// into 1 to 98 (inclusive).
	for (int i = 1; i < 99; i++)
	{
		mpcost_translation_table[ i ] = uint8_t( ((i-1)*251 -1 )/97+1 );
	}

	mu_gmheight = gmap.info.height ;
	mu_gmwidth  = gmap.info.width ;

	mf_ox = gmap.info.origin.position.x ;
	mf_oy = gmap.info.origin.position.x ;
	mf_resolution = gmap.info.resolution ;

	mcv_map = cv::Mat(mu_gmheight, mu_gmwidth, CV_8U);

	printf("\n input gmap info: \n %f %f %f %d %d \n", mf_ox, mf_oy, mf_resolution, mu_gmheight, mu_gmwidth);

	for (int iy=0; iy < mu_gmheight; iy++)
	{
	  for(int ix=0; ix < mu_gmwidth; ix++)
	  {
		  uint32_t iidx = iy * mu_gmwidth + ix ;
		  signed char val = gmap.data[iidx] ;
		  mcv_map.data[iidx] = val < 0 ? 127 : mpcost_translation_table[val];
	  }
	}

	//cv::Mat cvmap_orig = cvmap.clone();
}

VizSetCollector::~VizSetCollector()
{
	delete [] mpcost_translation_table;
}


void VizSetCollector::clusterToThreeLabels( cv::Mat& uImage  )
{
	cv::Mat uUnkn = uImage.clone();
	cv::threshold( uUnkn, uUnkn, 187, 255, cv::THRESH_TOZERO_INV ); 	// 187 ~ 255 --> 0
	cv::threshold( uUnkn, uUnkn, 67,  255, cv::THRESH_TOZERO ); 		// 0 ~ 66 	--> 0
	cv::threshold( uUnkn, uUnkn, 0, UNKNOWN, cv::THRESH_BINARY) ;// 67 ~ 187  --> 127 (unknown)

	cv::Mat uOcc = uImage.clone();
	cv::threshold( uOcc, uOcc, 128, OCCUPIED, cv::THRESH_BINARY ); // 187 ~ 255 --> 255
	uImage = uOcc + uUnkn ;
}


void VizSetCollector::initialize()
{
	mcv_map_orig = mcv_map.clone();
	if (mn_numpyrdownsample > 0)
	{
		for(int iter=0; iter < mn_numpyrdownsample; iter++ )
		{
			int nrows = mcv_map.rows; //% 2 == 0 ? img.rows : img.rows + 1 ;
			int ncols = mcv_map.cols; // % 2 == 0 ? img.cols : img.cols + 1 ;
			pyrDown(mcv_map, mcv_map, cv::Size( ncols/2, nrows/2 ) );
		}
	}

	clusterToThreeLabels( mcv_map );

//	cv::Mat cvimg;
//	cv::cvtColor(mcv_map, cvimg, cv::COLOR_GRAY2RGB);
	mu_cvheight = mcv_map.rows;
	mu_cvwidth  = mcv_map.cols;


	mo_costmap.resizeMap( mu_cvwidth, mu_cvheight, mf_resolution, 0, 0 );

	memcpy(mo_costmap.getCharMap(), &mcv_map.data[0], mu_cvheight*mu_cvwidth*sizeof(uchar) ) ;
	printf(" cv ds mapsize: (%d %d) \n", mu_cvheight, mu_cvwidth);

	///////////////////////////////////////////////////////////////////////////////////
	// 			1. Identify all free cells
	///////////////////////////////////////////////////////////////////////////////////

	for (int iy=0; iy < mu_cvheight; iy++)
	{
		for (int ix =0; ix < mu_cvwidth; ix++)
		{
			uint32_t iidx = iy * mu_cvwidth + ix ;
			uchar val = mcv_map.data[iidx] ;
			if(val == 0 )
			{
				VizSet oVG( mu_cvwidth, mu_cvheight, ix, iy, iidx );
				mve_FreeCells.push_back(oVG);
// 			  cv::circle(cvimg, cv::Point(jj,ii), 2, cv::Scalar(0,255,255), 1, 8, 0) ;
			}
		}
	}
}

void VizSetCollector::computeVisibiliies()
{
	  clock_t start = clock();
	  double rayshoot_time = 0;
	  int nradius = FOV_RADIUS / std::pow(2, mn_numpyrdownsample) ;
	  printf(" num pts to rayshoot: %lu \n rayshoot depth: %d \n", mve_FreeCells.size(), nradius);

#ifdef DEBUG_DRAW
cv::Mat cvimg;
cv::cvtColor(mcv_map, cvimg, cv::COLOR_GRAY2RGB);
#endif
	  for(int tmpidx = 0; tmpidx < mve_FreeCells.size(); tmpidx++)
	  {
		  VizSet oVG = mve_FreeCells[tmpidx] ;
		  uint32_t cellidx = oVG.GetSelfIdx() ;
		  MapLocation rstart = oVG.GetSelfLoc();

#ifdef DEBUG_DRAW
cv::circle(cvimg, cv::Point(rstart.x, rstart.y), 1, cv::Scalar(255,0,0), 1, 8, 0) ;
#endif
		  double perc = (double)tmpidx / (double)mve_FreeCells.size() * 100 ;
		  printf("\r %04f", perc); fflush(stdout);
		  for (int ii =0; ii < 360; ii++ )
		  {
			  costmap_2d::MapLocation rend ;
			  int X = static_cast<int>( (double)rstart.x + nradius * cos( ANG_STEP * ii ) ) ;
			  int Y = static_cast<int>( (double)rstart.y + nradius * sin( ANG_STEP * ii ) ) ;
			  rend.x = MIN(MAX(X, 0), mu_cvwidth);
			  rend.y = MIN(MAX(Y, 0), mu_cvheight);
			  std::vector<costmap_2d::MapLocation> polygon;
			  polygon.push_back(rstart);
			  polygon.push_back(rend);

	  std::vector<costmap_2d::MapLocation> polygon_cells;
	  clock_t rstart_time = clock();
	  mo_costmap.polygonOutlineCells( polygon, polygon_cells );
	  clock_t rend_time = clock();
	  rayshoot_time += double(rend_time - rstart_time)/CLOCKS_PER_SEC;

			  for (int jj=0; jj < polygon_cells.size(); jj++)
			  {
				  cv::Point pt( polygon_cells[jj].x, polygon_cells[jj].y ) ;
#ifdef DEBUG_DRAW
cv::circle(cvimg, pt, 2, cv::Scalar(0,255,255), 1, 8, 0) ;
#endif
				  uint32_t uchidx = pt.y * mu_cvwidth + pt.x ;
				  mve_FreeCells[tmpidx].appendMemberCellIdx( uchidx ) ;
				  mve_FreeCells[tmpidx].appendMemberCellLoc( polygon_cells[jj] );
			  }
			  //veFreeCells[tmpidx] = oVG ;
		  }
	  }
	  clock_t end = clock();
	  double elapsed = double(end - start)/CLOCKS_PER_SEC;
	  printf("\nRayshoot process time measured: %.3f seconds.\n", elapsed);
	  printf("Rayshoot time only: %.3f seconds. \n", rayshoot_time);

#ifdef DEBUG_DRAW
cv::namedWindow("rayshoot res", 0);
cv::imshow("rayshoot res", cvimg);
cv::waitKey(0);
cv::Mat cvimg_orig ;
cv::cvtColor(mcv_map_orig, cvimg_orig, cv::COLOR_GRAY2RGB);
#endif
	    clock_t upscale_start = clock();
	    // upsample freecells
	    if (mn_numpyrdownsample > 0)
	    {
	    	uint32_t nscale = std::pow(2, mn_numpyrdownsample) ;
			for(int idx=0; idx < mve_FreeCells.size() ; idx++)
			{
				// upsample self loc
				VizSet oVG = mve_FreeCells[idx];
				MapLocation upscaled_pt = oVG.GetSelfLoc() ;
				upscaled_pt.x = oVG.GetSelfLoc().x * nscale ;
				upscaled_pt.y = oVG.GetSelfLoc().y * nscale ;
				uint32_t upscaled_selfidx = upscaled_pt.y * mu_gmwidth + upscaled_pt.x ;

				// update self loc / idx
				mve_FreeCells[idx].SetSelfIdx( upscaled_selfidx) ;
				mve_FreeCells[idx].SetSelfLoc( upscaled_pt );

				// update member locs / idxs
				std::vector<MapLocation> mpts = mve_FreeCells[idx].GetMemberLocs() ;
				std::vector<uint32_t> lidxs = mve_FreeCells[idx].GetMemberIdxs() ;
				for ( int midx = 0; midx < mpts.size(); midx++)
				{
					MapLocation up_mp = mpts[midx];
					up_mp.x = up_mp.x * nscale ;
					up_mp.y = up_mp.y * nscale ;
					mpts[midx] = up_mp ;
					lidxs[midx] = up_mp.y * mu_gmwidth + up_mp.x  ;
#ifdef DEBUG_DRAW
cv::circle(cvimg_orig, cv::Point(up_mp.x, up_mp.y), 2, cv::Scalar(0,255,255), 1, 8, 0) ;
#endif
				}
				mve_FreeCells[idx].SetMemberLocs(mpts);
				mve_FreeCells[idx].SetMemberIdxs(lidxs) ;
			}
	    }
	clock_t upscale_end = clock();
	double upscaling_elapsed = double(upscale_end - upscale_start)/CLOCKS_PER_SEC;
	printf("Upscaling time measured: %.3f seconds.\n", upscaling_elapsed);

#ifdef DEBUG_DRAW
cv::namedWindow("upsample res", 0);
cv::imshow("upsample res", cvimg_orig);
cv::waitKey(0);
#endif
}



}
