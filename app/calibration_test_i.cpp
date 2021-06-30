#include <xyf_cal_lib.h>

int main(void)
{

    calibration::cm_xyf::MyStruct cm_data;

    cm_data.image_path = "G://calibration//test_4//test_4.bmp";
    cm_data.post_path_1 = "G://calibration//test_4//corneroutput_1.png";
    cm_data.post_path_2 = "G://calibration//test_4//image_distort_1.jpg";
    cm_data.corner_rows = 8;
    cm_data.corner_cols = 6;
    cm_data.cornersize_cols = 35;
    cm_data.cornersize_rows = 35;

	calibration::cm_xyf cm_xyf_instance(cm_data);
	cm_xyf_instance.getcmdata();
    return 0;

}