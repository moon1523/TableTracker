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


bool point_cloud_depth_to_color(k4a_transformation_t transformation_handle,
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
