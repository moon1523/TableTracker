#include "Functions.hh"





k4a_device_configuration_t get_default_config()
{
	k4a_device_configuration_t camera_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	camera_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	camera_config.color_resolution = K4A_COLOR_RESOLUTION_720P;
	camera_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	camera_config.camera_fps = K4A_FRAMES_PER_SECOND_15;
	camera_config.subordinate_delay_off_master_usec = 0;
	camera_config.synchronized_images_only = true;

	return camera_config;
}

cv::Mat color_to_opencv(const k4a_image_t im)
{
	cv::Mat cv_image_with_alpha(k4a_image_get_height_pixels(im), k4a_image_get_width_pixels(im), CV_8UC4, (void*)k4a_image_get_buffer(im));
    cv::Mat cv_image_no_alpha;
    cv::cvtColor(cv_image_with_alpha, cv_image_no_alpha, cv::COLOR_BGRA2BGR);
    return cv_image_no_alpha;
}

cv::Mat depth_to_opencv(const k4a_image_t im)
{
    return cv::Mat(k4a_image_get_height_pixels(im),
    			   k4a_image_get_width_pixels(im),
				   CV_16U,
        (void*)k4a_image_get_buffer(im),
        static_cast<size_t>(k4a_image_get_stride_bytes(im)));
}



k4a_image_t color_to_depth(k4a_transformation_t transformation_handle,
									   const k4a_image_t depth_image,
									   const k4a_image_t color_image)
{
    int depth_image_width_pixels = k4a_image_get_width_pixels(depth_image);
    int depth_image_height_pixels = k4a_image_get_height_pixels(depth_image);
    k4a_image_t transformed_color_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                                                 depth_image_width_pixels,
                                                 depth_image_height_pixels,
                                                 depth_image_width_pixels * 4 * (int)sizeof(uint8_t),
                                                 &transformed_color_image))
    {
        printf("Failed to create transformed color image\n");
        exit(1);
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_color_image_to_depth_camera(transformation_handle,
                                                                               depth_image,
                                                                               color_image,
                                                                               transformed_color_image))
    {
        printf("Failed to compute transformed color image\n");
        exit(1);
    }

    return transformed_color_image;
}

k4a_image_t create_depth_image_like(const k4a_image_t im)
{
	k4a_image_t img;
	k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
					 k4a_image_get_width_pixels(im),
					 k4a_image_get_height_pixels(im),
					 k4a_image_get_width_pixels(im) * static_cast<int>(sizeof(uint16_t)),
					 &img);
	return img;
}

k4a_image_t create_color_image_like(const k4a_image_t im)
{
	k4a_image_t img;
	k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
					 k4a_image_get_width_pixels(im),
					 k4a_image_get_height_pixels(im),
					 k4a_image_get_width_pixels(im) * 4 * static_cast<int>(sizeof(uint8_t)),
					 &img);
	return img;
}
k4a_image_t create_point_cloud_based_color(const k4a_image_t im)
{
	k4a_image_t img;

	k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
					 k4a_image_get_width_pixels(im),
					 k4a_image_get_height_pixels(im),
					 k4a_image_get_width_pixels(im) * 3 * (int)sizeof(int16_t),
					 &img);
	return img;
}

k4a_image_t create_point_cloud_based_depth(const k4a_image_t im)
{
	k4a_image_t img;

	k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
					 k4a_image_get_width_pixels(im),
					 k4a_image_get_height_pixels(im),
					 k4a_image_get_width_pixels(im) * 1 * (int)sizeof(int16_t),
					 &img);
	return img;
}


cv::Mat create_color_xy_table(const k4a_calibration_t calibration)
{
	k4a_float2_t p;
	k4a_float3_t ray;

	int width = calibration.color_camera_calibration.resolution_width;
	int height = calibration.color_camera_calibration.resolution_height;

	cv::Mat xy_table = cv::Mat::zeros(height,width, CV_32FC2);
	float* xy_table_data = (float*)xy_table.data;
	int valid;

	for (int y = 0, idx = 0; y < height; y++)
	{
		p.xy.y = (float)y;
		for (int x = 0; x < width; x++, idx++)
		{
			p.xy.x = (float)x;
			k4a_calibration_2d_to_3d(&calibration, &p, 1.f, K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_COLOR, &ray, &valid);
			if (valid) {
				xy_table_data[idx*2] = ray.xyz.x;
				xy_table_data[idx*2+1] = ray.xyz.y;
			} else {
				xy_table_data[idx*2] = nanf("");
				xy_table_data[idx*2+1] = nanf("");
			}
		}
	}
	return xy_table;
}

void transformation_helpers_write_point_cloud(const k4a_image_t point_cloud_image,
                                              const k4a_image_t color_image,
                                              const char *file_name)
{
    std::vector<color_point_t> points;

    int width = k4a_image_get_width_pixels(point_cloud_image);
    int height = k4a_image_get_height_pixels(color_image);

    int16_t *point_cloud_image_data = (int16_t *)(void *)k4a_image_get_buffer(point_cloud_image);
    uint8_t *color_image_data = k4a_image_get_buffer(color_image);

    for (int i = 0; i < width * height; i++)
    {
        color_point_t point;
        point.xyz[0] = point_cloud_image_data[3 * i + 0];
        point.xyz[1] = point_cloud_image_data[3 * i + 1];
        point.xyz[2] = point_cloud_image_data[3 * i + 2];
        if (point.xyz[2] == 0)
        {
            continue;
        }

        point.rgb[0] = color_image_data[4 * i + 0];
        point.rgb[1] = color_image_data[4 * i + 1];
        point.rgb[2] = color_image_data[4 * i + 2];
        uint8_t alpha = color_image_data[4 * i + 3];

        if (point.rgb[0] == 0 && point.rgb[1] == 0 && point.rgb[2] == 0 && alpha == 0)
        {
            continue;
        }

        points.push_back(point);
    }

#define PLY_START_HEADER "ply"
#define PLY_END_HEADER "end_header"
#define PLY_ASCII "format ascii 1.0"
#define PLY_ELEMENT_VERTEX "element vertex"

    // save to the ply file
    std::ofstream ofs(file_name); // text mode first
    ofs << PLY_START_HEADER << std::endl;
    ofs << PLY_ASCII << std::endl;
    ofs << PLY_ELEMENT_VERTEX << " " << points.size() << std::endl;
    ofs << "property float x" << std::endl;
    ofs << "property float y" << std::endl;
    ofs << "property float z" << std::endl;
    ofs << "property uchar red" << std::endl;
    ofs << "property uchar green" << std::endl;
    ofs << "property uchar blue" << std::endl;
    ofs << PLY_END_HEADER << std::endl;
    ofs.close();

    std::stringstream ss;
    for (size_t i = 0; i < points.size(); ++i)
    {
        // image data is BGR
        ss << (float)points[i].xyz[0] << " " << (float)points[i].xyz[1] << " " << (float)points[i].xyz[2];
        ss << " " << (float)points[i].rgb[2] << " " << (float)points[i].rgb[1] << " " << (float)points[i].rgb[0];
        ss << std::endl;
    }
    std::ofstream ofs_text(file_name, std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}

static bool point_cloud_depth_to_color(k4a_transformation_t transformation_handle,
                                       const k4a_image_t depth_image,
                                       const k4a_image_t color_image)
{
    // transform color image into depth camera geometry
    int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
    int color_image_height_pixels = k4a_image_get_height_pixels(color_image);

    k4a_image_t transformed_depth_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                                                 color_image_width_pixels,
                                                 color_image_height_pixels,
                                                 color_image_width_pixels * (int)sizeof(uint16_t),
                                                 &transformed_depth_image))
    {
        printf("Failed to create transformed depth image\n");
        return false;
    }

    k4a_image_t point_cloud_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                                                 color_image_width_pixels,
                                                 color_image_height_pixels,
                                                 color_image_width_pixels * 3 * (int)sizeof(int16_t),
                                                 &point_cloud_image))
    {
        printf("Failed to create point cloud image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED !=
        k4a_transformation_depth_image_to_color_camera(transformation_handle, depth_image, transformed_depth_image))
    {
        printf("Failed to compute transformed depth image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(transformation_handle,
                                                                              transformed_depth_image,
                                                                              K4A_CALIBRATION_TYPE_COLOR,
                                                                              point_cloud_image))
    {
        printf("Failed to compute point cloud\n");
        return false;
    }

    k4a_image_release(transformed_depth_image);
    k4a_image_release(point_cloud_image);

    return true;
}

k4a_image_t Convert_Color_MJPG_To_BGRA(k4a_image_t color_image)
{
	k4a_image_t uncompressed_color_image = NULL;
	// Convert color frame from mjpeg to bgra
	k4a_image_format_t format;
	format = k4a_image_get_format(color_image);
	if (format != K4A_IMAGE_FORMAT_COLOR_MJPG) {
		printf("Color format not supported. Please use MJPEG\n"); exit(1);
	}

	int color_width, color_height;
	color_width = k4a_image_get_width_pixels(color_image);
	color_height = k4a_image_get_height_pixels(color_image);

	if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
												 color_width,
												 color_height,
												 color_width * 4 * (int)sizeof(uint8_t),
												 &uncompressed_color_image))
	{
		printf("Failed to create image buffer\n"); exit(1);
	}

	tjhandle tjHandle;
	tjHandle = tjInitDecompress();
	if (tjDecompress2(tjHandle,
					  k4a_image_get_buffer(color_image),
					  static_cast<unsigned long>(k4a_image_get_size(color_image)),
					  k4a_image_get_buffer(uncompressed_color_image),
					  color_width,
					  0, // pitch
					  color_height,
					  TJPF_BGRA,
					  TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE) != 0)
	{
		printf("Failed to decompress color frame\n");
		if (tjDestroy(tjHandle))
		{
			printf("Failed to destroy turboJPEG handle\n");
		}
		exit(1);
	}
	if (tjDestroy(tjHandle))
	{
		printf("Failed to destroy turboJPEG handle\n");
	}
	return uncompressed_color_image;
}

pair<Quaterniond, Vector3d> ReadCharucoData(string fileName)
{
	// Read World Coordinate
	Quaterniond quat;
	Vector3d tvec;
	ifstream ifs(fileName);
	if(!ifs.is_open()) {
		cerr << "Fail to read the charuco data" << endl;
		exit(1);
	}
	string dump;

	ifs >> dump >> quat.x() >> quat.y() >> quat.z() >> quat.w();
	ifs >> dump >> tvec.x() >> tvec.y() >> tvec.z();

	return make_pair(quat, tvec);
}

tuple<Point2i, Point2i, Point2i> ReadTableCenterData(string fileName)
{
	Point2i p1, p2, center;
	ifstream ifs(fileName);
	if (!ifs.is_open()) {
		cerr << "Fail to read table center data" << endl;
		exit(1);
	}

	string dump;
	ifs >> dump >> p1.x >> p1.y;
	ifs >> dump >> p2.x >> p2.y;
	ifs >> dump >> center.x >> center.y;

	return make_tuple(p1, p2, center);
}
