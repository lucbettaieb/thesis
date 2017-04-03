/*
 * caffe_ros
 * 
 * An easier to use caffe/ROS bridge.  Inspired by "ros_caffe" by Tzutalin
 * (c) 2015 Tzutalin
 * (c) 2017 Luc Bettaieb
 * BSD
 */

#include <cnn_localization/caffe_ros.h>

static bool PairCompare(const std::pair<float, int>& lhs,
                        const std::pair<float, int>& rhs) {
    return lhs.first > rhs.first;
}

/* Return the indices of the top N values of vector v. */
static std::vector<int> Argmax(const std::vector<float>& v, int N) {
    std::vector<std::pair<float, int> > pairs;
    for (size_t i = 0; i < v.size(); ++i)
        pairs.push_back(std::make_pair(v[i], i));
    std::partial_sort(pairs.begin(), pairs.begin() + N, pairs.end(), PairCompare);

    std::vector<int> result;
    for (int i = 0; i < N; ++i)
        result.push_back(pairs[i].second);
    return result;
}

CaffeROS::CaffeROS(const string& model_file,
                       const string& trained_file,
                       const string& mean_file,
                       const string& label_file)
{
#ifdef CPU_ONLY
  Caffe::set_mode(Caffe::CPU);
#else
  Caffe::set_mode(Caffe::GPU);
#endif
  /* Load the network. */

  net_.reset(new caffe::Net<float>(model_file, caffe::TEST));

  net_->caffe::CopyTrainedLayersFrom(trained_file);

  caffe::Blob<float>* input_layer = net_->input_blobs()[0];
  num_channels_ = input_layer->channels();

  input_geometry_ = cv::Size(input_layer->width(), input_layer->height());

  /* Load the binaryproto mean file. */
  SetMean(mean_file);

  /* Load labels. */
  std::ifstream labels(label_file.c_str());
  string line;

  while (std::getline(labels, line))
  {
    labels_.push_back(string(line));
  }

  caffe::Blob<float>* output_layer = net_->output_blobs()[0];
}

std::vector<Prediction> CaffeROS::Classify(const cv::Mat& img, int N)
{
  std::vector<float> output = Predict(img);

  std::vector<int> maxN = Argmax(output, N);
  std::vector<Prediction> predictions;
  for (uint i = 0; i < N; ++i)
  {
    int idx = maxN[i];
    predictions.push_back(std::make_pair(labels_[idx], output[idx]));
  }

  return predictions;
}

void CaffeROS::SetMean(const string& mean_file)
{
  caffe::BlobProto blob_proto;
  caffe::ReadProtoFromBinaryFileOrDie(mean_file.c_str(), &blob_proto);

  /* Convert from BlobProto to Blob<float> */
  caffe::Blob<float> mean_blob;
  mean_blob.FromProto(blob_proto);

  /* The format of the mean file is planar 32-bit float BGR or grayscale. */
  std::vector<cv::Mat> channels;

  float* data = mean_blob.mutable_cpu_data();

  for (int i = 0; i < num_channels_; ++i)
  {
    /* Extract an individual channel. */
    cv::Mat channel(mean_blob.height(), mean_blob.width(), CV_32FC1, data);
    channels.push_back(channel);
    data += mean_blob.height() * mean_blob.width();
  }
  /* Merge the separate channels into a single image. */
  cv::Mat mean;
  cv::merge(channels, mean);

  /* Compute the global mean pixel value and create a mean image
   * filled with this value. */
  cv::Scalar channel_mean = cv::mean(mean);
  mean_ = cv::Mat(input_geometry_, mean.type(), channel_mean);
}

std::vector<float> CaffeROS::Predict(const cv::Mat& img)
{
  cafe::Blob<float>* input_layer = net_->input_blobs()[0];
  input_layer->Reshape(1, num_channels_,
                       input_geometry_.height, input_geometry_.width);
  /* Forward dimension change to all layers. */
  net_->Reshape();

  std::vector<cv::Mat> input_channels;
  WrapInputLayer(&input_channels);

  Preprocess(img, &input_channels);

  net_->ForwardPrefilled();

  /* Copy the output layer to a std::vector */
  Blob<float>* output_layer = net_->output_blobs()[0];
  const float* begin = output_layer->cpu_data();
  const float* end = begin + output_layer->channels();
  return std::vector<float>(begin, end);
}

/* Wrap the input layer of the network in separate cv::Mat objects
 * (one per channel). This way we save one memcpy operation and we
 * don't need to rely on cudaMemcpy2D. The last preprocessing
 * operation will write the separate channels directly to the input
 * layer. */
void CaffeROS::WrapInputLayer(std::vector<cv::Mat>* input_channels) {
  caffe::Blob<float>* input_layer = net_->input_blobs()[0];

  int width = input_layer->width();
  int height = input_layer->height();
  float* input_data = input_layer->mutable_cpu_data();
  for (int i = 0; i < input_layer->channels(); ++i)
  {
    cv::Mat channel(height, width, CV_32FC1, input_data);
    input_channels->push_back(channel);
    input_data += width * height;
  }
}

void CaffeROS::Preprocess(const cv::Mat& img,
                            std::vector<cv::Mat>* input_channels) {
  /* Convert the input image to the input image format of the network. */
  cv::Mat sample;
  if (img.channels() == 3 && num_channels_ == 1)
  {
    cv::cvtColor(img, sample, CV_BGR2GRAY);
  }
  else if (img.channels() == 4 && num_channels_ == 1)
  {
    cv::cvtColor(img, sample, CV_BGRA2GRAY);
  }
  else if (img.channels() == 4 && num_channels_ == 3)
  {
    cv::cvtColor(img, sample, CV_BGRA2BGR);
  }
  else if (img.channels() == 1 && num_channels_ == 3)
  {
    cv::cvtColor(img, sample, CV_GRAY2BGR);
  }
  else
  {
    sample = img;
  }

  cv::Mat sample_resized;
  if (sample.size() != input_geometry_)
  {
    cv::resize(sample, sample_resized, input_geometry_);
  }
  else
  {
    sample_resized = sample;
  }

  cv::Mat sample_float;
  if (num_channels_ == 3)
  {
    sample_resized.convertTo(sample_float, CV_32FC3);
  }
  else
  {
    sample_resized.convertTo(sample_float, CV_32FC1);
  }

  cv::Mat sample_normalized;
  cv::subtract(sample_float, mean_, sample_normalized);

  /* This operation will write the separate BGR planes directly to the
   * input layer of the network because it is wrapped by the cv::Mat
   * objects in input_channels. */
  cv::split(sample_normalized, *input_channels);
}
