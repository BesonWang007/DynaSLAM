/**
* This file is part of DynaSLAM.
* Copyright (C) 2018 Berta Bescos <bbescos at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/bertabescos/DynaSLAM>.
* mask-rcnn 实现的语义分割= 
主要是 python接口=
需要做一些数据转换====
*/

#ifndef __MASKNET_H
#define __MASKNET_H

#ifndef NULL
#define NULL   ((void *) 0)
#endif

#include <python2.7/Python.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <cstdio>
#include <boost/thread.hpp>
#include "include/Conversion.h"

namespace DynaSLAM
{

class SegmentDynObject{
private:
	NDArrayConverter *cvt; 	/*!<  cv::Mat  转换到  NumPy Array  */
	PyObject *py_module; 	/*!< python 模块 Module of python where the Mask algorithm is implemented */
	PyObject *py_class; 	        /*!< Class to be instanced */
	PyObject *net; 			/*!< Instance of the class */
	std::string py_path; 	        /*!< PYTHONPATH 变量路径 */
	std::string module_name; /*!< Detailed description after the member */
	std::string class_name; /*!< Detailed description after the member */
        std::string get_dyn_seg; 	/*!< Detailed description after the member */

	void ImportSettings();
public:

	SegmentDynObject();
        ~SegmentDynObject();
// 获取检测结果 语义分割结果=======
        cv::Mat GetSegmentation(cv::Mat &image, std::string dir="no_save", std::string rgb_name="no_file");
};


}

#endif
