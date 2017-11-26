#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <chrono>

ros::Time lastFrameTime;
ros::Publisher debug_pub;

typedef unsigned char u8;
typedef uint16_t u16;

typedef struct {
  int w, h;
  u8* data;
} outimage;

typedef struct {
  int w, h;
  u16* data;
} image;

typedef struct {
  int len;
  image** items;
} imagelist;


void calc_stats(imagelist* iml, float** mean, float** stdev) {
  int x;
  int y;
  int i;
  int w;
  int h;
  int sum;
  int pi;
  int nframes;
  float avg;
  float err;
  float sqerr;

  w = iml->items[0]->w;
  h = iml->items[0]->h;

  nframes = iml->len;

  *mean = (float*) malloc(w * h * sizeof(float));
  *stdev = (float*) malloc(w * h * sizeof(float));

  auto start = std::chrono::high_resolution_clock::now();

  for (y = 0; y < h; y++) {
    for (x = 0; x < w; x++) {
      pi = y * w + x;
      sum = 0;
      for (i = 0; i < nframes; i++) {
        sum += iml->items[i]->data[pi];
      }
      avg = (float) sum / (float) nframes;
      sqerr = 0;
      for (i = 0; i < nframes; i++) {
        err = (float) iml->items[i]->data[pi] - avg;
        sqerr += err * err;
      }
      sqerr = sqerr / (float) nframes;
      (*mean)[pi] = avg;
      (*stdev)[pi] = sqrt(sqerr);
    }
  }

  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> tot_t = end - start;
  std::cout << "tot_t time: " << tot_t.count() * 1000 << " ms\n";

}

float clamp(float v, float min, float max) {
  return v < min ? min : v > max ? max : v;
}

void ringbuffer_insert(imagelist *iml, image* img, int *counter) {
  image *prev = iml->items[*counter];
  if (prev) {
    free(prev->data);
    free(prev);
  }
  iml->items[*counter] = img;
  *counter = (*counter + 1) % iml->len;

}



// int main(int argc, char** argv) {
//   imagelist* iml;
//   float* mean;
//   float* stdev;
//   float* ema;
//   int i;
//   int x, y;
//   int w, h;
//   char path[100];
//   image out;
//   float dev;
//   int pi;
//   float v;
//   int lasti;
//   float signal;

//   iml = read_all(argc - 1, argv + 1);
//   calc_stats(iml, &mean, &stdev);

//   w = iml->items[0]->w;
//   h = iml->items[0]->h;
//   ema = (float*) malloc(w*h*sizeof(float));

//   out.w = w;
//   out.h = h;
//   out.data = (u8*) malloc(w*h*sizeof(u8));

//   for (y = 0; y < h; y++) {
//     for (x = 0; x < w; x++) {
//       ema[y*w + x] = 0;
//     }
//   }

//   mkdir("out", 0755);

//   lasti = 0;
//   for (i = 0; i < iml->len; i++) {
//     sprintf(path, "out/out-%04d.pgm", i);
//     for (y = 0; y < h; y++) {
//       for (x = 0; x < w; x++) {
//         pi = y*w + x;
//         v = (
//             (int) iml->items[i]->data[pi] +
//             (int) iml->items[lasti]->data[pi]
//         )*0.5;
//         /* Estimate the deviation of each pixel from its mean. */
//         dev = fabs(v - mean[pi]) / clamp(stdev[pi], 255*0.02, 255);
//         signal = dev > 1 ? fabs(dev - ema[pi]) : 0;

//         /* Take an exponential moving average of the deviation. */
//         ema[pi] = ema[pi]*0.9 + dev*0.1;

//         out.data[pi] = signal < 0 ? 0 : signal*60 > 255 ? 255 : signal*60;
//       }
//     }
//     write_pgm(path, &out);
//     lasti = i;
//   }
// }

imagelist *global_iml;
int iml_len;
int imagelist_ring_counter;
int images_received;

void global_init() {
  iml_len = 200;
  imagelist_ring_counter = 0;
  images_received = 0;

  global_iml = (imagelist*) malloc(sizeof(imagelist));
  global_iml->len = iml_len;
  global_iml->items = (image**) malloc(iml_len * sizeof(image*));
  for (int i = 0; i < iml_len; i++) {
    global_iml->items[i] = NULL;
  }
}



int neg_mod(int v, int mod) {
  if (v < 0) {
    return mod + v;
  }
  return v % mod;
}

void image_callback(const sensor_msgs::Image::ConstPtr& msg)
{
  float* mean;
  float* stdev;
  float* ema;
  image* in;
  outimage out;
  int lasti_;
  int i_;
  int x;
  int y;
  int pi;
  u16 *c_data;
  float dev;
  float v;
  float signal;

  auto start = std::chrono::high_resolution_clock::now();


  // Image data is 14 bits stored in 16 bits
  int w = msg->width;
  int h = msg->height;
  std::vector<uint8_t> data = msg->data;
  c_data = (u16*)&data[0];

  in = (image*) malloc(sizeof(image));
  in->w = w;
  in->h = h;
  in->data = (u16*) malloc(w * h * sizeof(u16));
  memcpy(in->data, c_data, w * h * sizeof(u16));
  ringbuffer_insert(global_iml, in, &imagelist_ring_counter);

  images_received++;
  if (images_received < iml_len) {
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> tot_t = end - start;
    return;
  }





  // Calculate time since last frame
  // Since we use the frame timestamps instead of the system time,
  // this will play nice with bag replay! e.g when we replay at half speed
  ros::Time timestamp = msg->header.stamp;
  ros::Duration sinceLastFrame = timestamp - lastFrameTime;
  lastFrameTime = timestamp;
  float deltaMs = sinceLastFrame.toSec() * 1000;

  calc_stats(global_iml, &mean, &stdev);

  ema = (float*) malloc(w * h * sizeof(float));

  out.w = w;
  out.h = h;
  out.data = (u8*) malloc(w * h * sizeof(u8));

  for (y = 0; y < h; y++) {
    for (x = 0; x < w; x++) {
      ema[y * w + x] = 0;
    }
  }



  // shorthand
  imagelist *iml = global_iml;

  // TODO: make ema calculation running, instead of recomputing each time
  lasti_ = 0;
  for (i_ = 0; i_ < iml->len; i_++) {
    int i = neg_mod(imagelist_ring_counter - 1 + i_, iml_len);
    int lasti = neg_mod(imagelist_ring_counter - 1 + lasti_, iml_len);

    for (y = 0; y < h; y++) {
      for (x = 0; x < w; x++) {
        pi = y * w + x;
        v = (
              (int) iml->items[i]->data[pi] +
              (int) iml->items[lasti]->data[pi]
            ) * 0.5;
        /* Estimate the deviation of each pixel from its mean. */
        dev = fabs(v - mean[pi]) / clamp(stdev[pi], 255 * 0.02, 255);
        signal = dev > 1 ? fabs(dev - ema[pi]) : 0;

        /* Take an exponential moving average of the deviation. */
        ema[pi] = ema[pi] * 0.9 + dev * 0.1;

        out.data[pi] = signal < 0 ? 0 : signal * 60 > 255 ? 255 : signal * 60;
      }
    }
    lasti_ = i_;
  }

  free(mean);
  free(stdev);
  free(ema);
  free(out.data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thermal_processing_node");

  global_init();

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/thermal/image_raw", 1000, image_callback);
  debug_pub = n.advertise<sensor_msgs::Image>("/thermal/debug", 1000);

  ros::spin();

  return 0;
}
