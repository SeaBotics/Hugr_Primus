
#include <pluginlib/class_list_macros.hpp>
#include "hugr_rviz_panel/hugr_camera_subscriber_panel.hpp"
#include <functional>
#include <QVBoxLayout>
#include <image_transport/transport_hints.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include <memory>
#include <rclcpp/qos.hpp>


namespace hugr_rviz_panel{

    HugrCameraPanel::HugrCameraPanel(QWidget* parent)
    : rviz_common::Panel(parent){
        QVBoxLayout* layout = new QVBoxLayout;

        //Dette konfigurer front end layout and position.
        video_label_ = new QLabel("Waiting for Image...");
        video_label_->setMinimumSize(320, 240);
        video_label_->setAlignment(Qt::AlignCenter);
        video_label_->setScaledContents(true);
        topic_input_ = new QLineEdit("camera/image_raw");
        
        // When user hits enter in the input box, calls updateTopic()
        connect(topic_input_, &QLineEdit::returnPressed, this, &HugrCameraPanel::updateTopic);
        auto topic_label = new QLabel("Topic: ");
        topic_label->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
        layout->addWidget(topic_label);
        layout->addWidget(topic_input_);
        layout->addWidget(video_label_);
        setLayout(layout);
    }



    void HugrCameraPanel::onInitialize(){
        rviz_common::Panel::onInitialize();

        auto node_ptr = getDisplayContext()->getRosNodeAbstraction().lock();
        if(node_ptr){
            node_ = node_ptr->get_raw_node();
            it_ = std::make_shared<image_transport::ImageTransport>(node_);


            updateTopic();
        }

    }


    //Denne seksjonen gjør kamera-topics konfigurerbar
    void HugrCameraPanel::updateTopic(){
        QString topic = topic_input_->text();
        std::string new_topic = topic.toStdString();
        if (new_topic.empty() || !it_ || new_topic == current_topic_) {return;} 

        sub_.shutdown();

        try{

        
         image_transport::TransportHints hints(node_.get(), "raw");

        sub_ = it_->subscribe(
            topic.toStdString(), // Converts UI text to a standard string ROS understands
            1,
            std::bind(&HugrCameraPanel::imageCallback, this, std::placeholders::_1),
            nullptr,
            &hints
        );

        current_topic_ = new_topic;
        RCLCPP_INFO(node_->get_logger(), "Subscribed to topic: %s", topic.toStdString().c_str());

            
        }catch (const std::exception& e){
            RCLCPP_ERROR(node_->get_logger(), "Failed to subscribe: %s", e.what());
        }

    }

    //VIKTIG DEL: Konverterer ROS image message til QImage og oppdatterer UI-en trygt
    void HugrCameraPanel::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg){

        if (msg->encoding != sensor_msgs::image_encodings::BGR8){
            RCLCPP_ERROR(rclcpp::get_logger("hugr_camera_panel"), "Unsupported encoding: %s", msg->encoding.c_str());
            return;
        }
        
        static int skip_count = 0;
        if (skip_count++ % 5 !=0) return;
        try{
                auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
                if(cv_ptr->image.empty()){
                    return;
                }

                QImage qimg(cv_ptr->image.data, 
                            cv_ptr->image.cols,
                            cv_ptr->image.rows,
                            cv_ptr->image.step,
                            QImage::Format_BGR888);


                QImage deepCopy = qimg.copy();

                QMetaObject::invokeMethod(
                    video_label_,
                    [this, deepCopy](){
                        video_label_->setPixmap(QPixmap::fromImage(deepCopy));
                    }, 
                    Qt::QueuedConnection
                );


        } catch (cv_bridge::Exception& e){
            RCLCPP_ERROR(node_->get_logger(), "Conversion error: %s", e.what());
        }
    }


    //Dette lagrer topics-navn konfigurer.
    void HugrCameraPanel::save(rviz_common::Config config) const{
        rviz_common::Panel::save(config);

        config.mapSetValue("topic", topic_input_->text());
    }

    void HugrCameraPanel::load(const rviz_common::Config& config){
        rviz_common::Panel::load(config);
        QString topic;

        if(config.mapGetString("topic", &topic)){
            topic_input_->setText(topic);
            updateTopic();
        }
    }
   

    //VIKITG: Dette gjør at rviz ser panellen. 
} PLUGINLIB_EXPORT_CLASS(hugr_rviz_panel::HugrCameraPanel, rviz_common::Panel)

