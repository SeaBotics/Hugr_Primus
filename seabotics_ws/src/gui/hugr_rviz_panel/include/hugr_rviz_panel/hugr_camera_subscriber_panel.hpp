#ifndef CAMERA_SUBSCRIBER_PANEL_HPP_
#define CAMERA_SUBSCRIBER_PANEL_HPP_
#pragma once


#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <QLabel>
#include <QPixmap>
#include <QImage>
#include <QLineEdit>
#include <QVBoxLayout>





namespace hugr_rviz_panel{


    class HugrCameraPanel : public rviz_common::Panel{
        Q_OBJECT
        public:
        HugrCameraPanel(QWidget* parent = nullptr);
        void onInitialize() override;

        virtual void load(const rviz_common::Config& config) override;
        virtual void save(rviz_common::Config config) const override;
        private slots:
        void updateTopic();



        private:
        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
        QLineEdit* topic_input_;
        QLabel* video_label_;

        std::string current_topic_;
 
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<image_transport::ImageTransport> it_;
        image_transport::Subscriber sub_;


    };



} // namespace hugr_rviz_panel


#endif