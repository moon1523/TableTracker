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



