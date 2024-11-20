#include <laser_camera_cal.h>    

void ResizeImage(cv::Mat& imagesrc, cv::Mat& imagedst)
{
    double fScale = 0.4;//缩放系数
    //计算目标图像的大小
    cv::Size dsize = cv::Size(imagesrc.cols*fScale, imagesrc.rows*fScale);
    cv::resize(imagesrc, imagedst, dsize);
}


int main(void)
{
    YAML::Node config = YAML::LoadFile("../config/config.yaml");
    std::string fold_path = config["fold_path"].as<std::string>();

    std::vector<cv::String> filenames;
    cv::glob(fold_path, filenames); 
    cv::Mat src_img, edge, dst_img, show;

    if(filenames.size() == 0)
    {
        std::cout << "File names size: " << filenames.size() << std::endl;
        return 0;
    }

    for(size_t i = 0; i < filenames.size(); ++i)
    {
        std::cout<< "File name: " << filenames[i] << std::endl;
        src_img = cv::imread(filenames[i], cv::IMREAD_GRAYSCALE);

        if(!src_img.data)
            std::cerr << "Problem loading image!!!" << std::endl;
        
        cv::threshold(src_img, edge, 150.0, 255, cv::THRESH_BINARY); 
        ResizeImage(edge, show);
        cv::namedWindow("center_point");
        cv::imshow("center_point", show);
        cv::waitKey(0);

        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(edge, dst_img, cv::MORPH_CLOSE, element);

        ResizeImage(dst_img, show);
        cv::namedWindow("center_point");
        cv::imshow("center_point", show);
        cv::waitKey(0);

        std::vector<cv::Point> points;

        for(int i = 0; i < dst_img.cols; i++)
        {
            float sum_pix = 0, sum_cols_pix = 0;
            for(int j = 0; j < dst_img.rows; j++)
            {
                float pixel = dst_img.data[i + j* dst_img.cols];
                sum_pix += pixel;
                sum_cols_pix += pixel* j;
            }
            float ave_rows = 0;
            ave_rows = sum_cols_pix / sum_pix;
            
            cv::circle(dst_img, cv::Point(i, ave_rows), 2, cv::Scalar(155));
            points.push_back(cv::Point(i, ave_rows));
        }
        ResizeImage(dst_img, dst_img);
        cv::namedWindow("center_point");
        cv::imshow("center_point", dst_img);
        cv::waitKey(0);
    }

    return 0;
}