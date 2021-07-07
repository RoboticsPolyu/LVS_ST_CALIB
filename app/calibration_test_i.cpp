#include <xyf_cal_lib.h>
#include "light_cal.h"

int main(void)
{
    calibration::cm_xyf::MyStruct cm_data;

    /* internal paramater calibration*/
    cm_data.fold_path = "/home/yang/image_fold";
    cm_data.corner_rows = 8;
    cm_data.corner_cols = 6;
    cm_data.cornersize_cols = 35;
    cm_data.cornersize_cols = 35;

	calibration::cm_xyf cm_xyf_instance(cm_data);
	cm_xyf_instance.getcmdata();

    cm_xyf_instance.GetImageCorners();
    
    /* */
    return 0;
}