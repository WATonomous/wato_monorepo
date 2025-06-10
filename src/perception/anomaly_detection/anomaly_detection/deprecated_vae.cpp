#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class SensorSubscriber :: public rclcpp::Node {
    public:
        SensorSubscriber() : Node("sensor_subscriber") {
            RCLCPP_INFO(this->get_logger(), "Initializing Sensor Subscriber Node");

            camera_subscriber = this->create_subscription<sensor_msgs::msg::Image>("/camera_data", 10,
            std::bind(&SensorSubscriber::camera_callback, this, std::placeholders::_1));

            lidar_subscriber = this->create_subscriber<sensor_msgs::msg::PointCloud2>("/lidar", 10, 
            std::bind(&SensorSubscriber::lidar_callback, this, std::placeholders::_1));
        }

    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscriber;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscriber;

        void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Receiving Image...");

            try {
                cv::Mat cv_image = convert_ros_to_cv(msg);
                
                cv::Mat processed_image = preprocess_camera_image(cv_image);

            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Error processing camera image: %s", e.what());
            }

            if(detect_camera_anomaly(processed_iamge)) {
                RCLCPP_WARN(this->get_logger(), "Camera anomaly detected: Overexposure or corruption.");
            }
        }

        cv::Mat convert_ros_to_cv(const sensor_msgs::msg::Image::SharedPtr msg) {

            cv_bridge::CVImagePtr cv_ptr;

            try {
                cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
            } catch (cv_bridge::Exception &e) {
                throw std::runtime_error("CV Bridge conversion failed: " + std::string(e.what()));
            }

            return cv_ptr->image;
        }

        void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Received LiDAR data! Processing...");

            try {
                std::vector<float> lidar_points = convert_ros_to_lidar(msg);

                std::vector<float> processed_lidar = preprocess_lidar_data(lidar_points);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Error processing LiDAR data: %s", e.what());
            }

            if (detect_lidar_noise(processed_lidar)) {
                RCLCPP_WARN(this->get_logger(), "LiDAR anomaly detected: Possible noise from rain or blur.");
            }
        }

        std::vector<float> convert_ros_to_lidar(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            std::vector<float> lidar_points;

            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

            for(; iter_x != iter_x.end(); iter_x++, iter_y++, iter_z++) {
                lidar_points.push_back(*iter_x);
                lidar_points.push_back(*iter_y);
                lidar_points.push_back(*iter_z);
            }

            return lidar_points;
        }

        std::vector<float> preprocess_lidar_data(const std::vector<float>& lidar_points) {
            std::vector<float> processed_lidar;
            size_t num_points = lidar_points.size() / 3;

            float mean_x = 0, mean_y = 0, mean_z = 0;

            for(size_t i = 0; i < lidar_points.size(); i += 3) {
                mean_x += lidar_points[i];
                mean_y += lidar_points[i + 1];
                mean_z += lidar_points[i + 2];
            }

            mean_x /= num_points;
            mean_y /= num_points;
            mean_z /= num_points;

            for(size_t i = 0; i < lidar.size(); i += 3) {
                processed_lidar.push_back((lidar_points[i] - mean_x) / 100.0f);
                processed_lidar.push_back((lidar_points[i + 1] - mean_y) / 100.0f);
                processed_lidar.push_back((lidar_points[i + 2] - mean_z) / 100.0f);
            }

            return processed_lidar;
        }

        bool detect_lidar_noise(const std::vector<float>& lidar_points) {
            if(lidar_points.empty()) return false;

            float mean_distance = 0.0f;
            float variance = 0.0f;
            size_t num_points = lidar_points.size() / 3;

            for(int i = 0; i < lidar_points.size(); i += 3) {
                float distance = std::sqrt(lidar_points[i] * lidar_points[i] + lidar_points[i + 1] * lidar_points[i + 1] + lidar_points[i + 2] * lidar_points[i + 2]);
                mean_distance += distance;
            }

            mean_distance /= num_points;

            for(int i = 0; i < lidar_points.size(); i += 3) {
                float distance = std::sqrt(lidar_points[i] * lidar_points[i] + lidar_points[i + 1] * lidar_points[i + 1] + lidar_points[i + 2] * lidar_points[i + 2]);

                variance += std::pow(distance - mean_distance / 2);
            }

            variance /= num_points;

            RCLCPP_INFO(rclcpp::get_logger("sensor_subscriber"), "LiDAR variance: %f", variance);

            return variance > 0.5f;
        }

        bool detect_camera_anomaly(const cv::Mat& image) {
            if(image.empty()) {
                RCLCPP_INFO(rclcpp::get_logger("sensor_subscriber"), "Received empty camera frame!");
                return true;
            }

            double mean_intensity = cv::mean(image)[0];

            RCLCPP_INFO(rclcpp::get_logger("sensor_subscriber"), "Camera mean intensity: %f", mean_intensity);

            return mean_intensity > 250.0;
        }

        cv::Mat pre_process_camera_image(cv::Mat image) {
            cv::resize(image, image, cv::Size(128, 128));
            cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
            image.convertTo(image, CV_32F, 1.0/255.0);

            return image;
        }

        torch::Tensor convert_image_to_Tensor(const cv::Mat& image) {
            
            // Converts OpenCV cv::Mat into a PyTorch tensor with shape (1, 1, height, width)
            // 	Provides raw pixel data, Reshapes data into a 4D tensor (batch, channels, height, width), Converts to float
            torch::Tensor tensor_image = torch::from_blob(image.data, {image.rows, image.cols}, torch::kFloat);

            // Normalizes pixel values to range [-1, 1] (ideal for deep learning)
            tensor_image = tensor_image * 2.0 - 1.0;

            return tensor_image;
        }

        torch::Tensor convert_lidar_to_tensor(const std::vector<float>& lidar_data) {

            // Converts a 1D LiDAR vector into a Torch tensor - Extracts raw LiDAR 3D coordinates, Organizes the points into a 1D tensor, Converts to Float
            torch::Tensor tensor_lidar = torch::from_blob((void*)lidar_data.data(), {1, (int)lidar_data.size()}, torch::kFloat);

            return tensor_lidar;
        }

}

struct Encoder : torch::nn::Module {
    torch::nn::Linear fc1{nullptr}, fc21{nullptr}, fc22{nullptr};

    Encoder(int input_dim, int hidden_dim, int layer_dim) {
        fc1 = register_module("fc1", torch::nn::Linear(input_dim, hidden_dim));
        fc2_mu = register_module("fc2_mu", torch::nn::Linear(hidden_dim, layer_dim));
        fc2_logvar = register_module("fc2_logvar", torch::nn::Linear(hidden_dim, layer_dim));
    }

    std::pair<torch::Tensor, torch::Tensor> forward(torch::Tensor x) {
        x = torch.relu(fc1(x));
        torch::Tensor mu = fc2_mu(x);
        torch::Tensor logvar = fc2_logvar(x);
        return {mu, logvar};
    }

    torch::Tensor reparametrize(torch::Tensor mu, torch::Tensor logvar) {
        torch::Tensor std = torch::exp(0.5 * logvar);
        torch::Tensor eps = torch::randn_like(std);
        return mu + eps * std;
    }
}

struct Decoder : torch::nn::Module {
    torch::nn::Linear fc1{nullptr}, fc2{nullptr};

    Decoder(torch::Tensor latent_dim, torch::Tensor hidden_dim, torch::Tensor output_dim) {
        fc1 = register_module("fc1", torch::nn::Linear(latent_dim, hidden_dim));
        fc2 = register_module("fc2", torch::nn::Linear(hidden_dim, output_dim));
    }

    torch::Tensor forward(torch::Tensor z) {
        z = torch::relu(fc1(z));
        return torch::sigmoid(fc2(z));
    }

    torch::Tensor vae_loss(torch::Tensor recon_x, torch::Tensor x, torch::Tensor mu, torch::Tensor logvar) {
        torch::Tensor recon_loss = torch::binary_cross_entropy(recon_x, x, torch::Reduction::Sum);
        torch::Tensor kl_divergence = 0.5 * torch::sum(1 + logvar - mu.pow(2) - torch::exp(logvar));

        return recon_loss + kl_divergence;
    }
}

struct VAE : torch::nn::Module {
    Encoder encoder{nullptr};
    Decoder decoder{nullptr};

    VAE(int input_dim, int hidden_dim, int latent_dim, int output_dim) : encoder(input_dim, hidden_dim, latent_dim), decoder(latent_dim, hidden_dim, output_dim) {
        encoder = register_module("encoder", Encoder(input_dim, hidden_dim, latent_dim));
        decoder = register_module("decoder", Decoder(latent_dim, hidden_dim, output_dim));
    }

    torch::Tensor forward(torch::Tensor x) {
        auto [mu, logvar] = encoder(x);
        tensor::Torch z = reparametrize(mu, logvar);
        return decoder(z);
    }
}

void vae(VAE& model, torch::data::DataLoader<torch::data::datasets::MNIST>& data_loader, int epochs, torch::optim::Adam& optimizer) {
    model.train();

    for(int epoch = 0; epoch < epochs; epoch++) {
        for(auto& batch : *data_loader) {
            torch::Tensor x = batch.data.view({batch.data.size(0), 784}).to(torch::kF32) / 255.0;
            torch::Tensor y = x.clone();

            optimizer.zero_grad();

            auto [mu, logvar] = model.encoder(x);
            tensor::Torch z = model.encoder.reparametrize(mu, logvar);
            torch::Tensor recon_x = decoder(z);

            torch::Tensor loss = vae_loss(recon_x, y, mu, logvar);
            loss.backward();
            optimize.step();

            total_loss += loss.item<float>();
        }

        std::cout << "Epoch [" << epoch + 1 << "/" << epochs << "], Loss: " << total_loss / data_loader.size() << std::endl;
    }
}

torch::Tensor generate_samples(VAE& model, int num_samples) {
    torch::Tensor z = torch::randn({num_samples, 20});
    z = z.to(torch::kF32);
    torch::Tensor samples = model.decoder(z);
    return samples;
}

int main() {
    // Hyperparameters
    int input_dim = 784;  // e.g., for flattened 28x28 image
    int hidden_dim = 400;
    int latent_dim = 20;
    int output_dim = 784;  // Output size same as input size for reconstruction

    // Create the VAE model
    VAE model(input_dim, hidden_dim, latent_dim, output_dim);

    // Dummy input (e.g., a batch of images)
    torch::Tensor input = torch::randn({64, input_dim});  // Batch of 64, each with 784 features

    // Forward pass through the VAE
    torch::Tensor output = model.forward(input);

    // Compute loss
    auto [mu, logvar] = model.encoder(input);
    torch::Tensor loss = vae_loss(output, input, mu, logvar);

    std::cout << "VAE Loss: " << loss.item<float>() << std::endl;

    auto dataset = torch::data::datasets::MNIST("./data").map(
        torch::data::transforms::Stack<>());
    
    auto data_loader = torch::data::make_data_loader(
        std::move(dataset), torch::data::DataLoaderOptions().batch_size(64));

    torch::optim::Adam optimizer(model.parameters(), torch::optim::AdamOptions(1e-3));

    train_vae(model, *data_loader, 10, optimizer);

    return 0;
}