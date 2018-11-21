#include <Yolo.h>

#define DAT_FILE    (char*)"/home/nknight/catkin_ws/src/yolo_ts_detector/darknet/trafficsign/ts.data"
#define CFG_FILE    (char*)"/home/nknight/catkin_ws/src/yolo_ts_detector/darknet/trafficsign/ts.cfg"
#define WEI_FILE    (char*)"/home/nknight/catkin_ws/src/yolo_ts_detector/darknet/backup/ts.backup"

#define NUM_CLAS    2

using namespace cv;

Yolo::Yolo(){
    char* datacfg = DAT_FILE;
    
    /* Load configuration */
    list *options = read_data_cfg(datacfg);
    char *name_list = option_find_str(options, (char*)"names", (char*)"data/names.list");
    names = get_labels(name_list);   
    alphabet = load_alphabet();
}

Yolo::~Yolo(){
    free_network(net);
}

void Yolo::loadNetwork(){
    char* cfgfile = CFG_FILE;
    char* weightfile = WEI_FILE;
    
    net = load_network(cfgfile, weightfile, 0);
    printf("Number of layer: %d \n", net->n);
    set_batch_network(net, 1);
    srand(2222222);
    output_layer = net->layers[net->n-1];
}

/**
 * @brief Make a prediction with YOLO
 * @param x Input opencv image type Mat
 * @return vector<YoloObjects> A vector of detected YOLO object
 */
vector<YoloObject> Yolo::predict(Mat x){
    vector<YoloObject> yoloObjects;

    /* Preprocessing */
    image im = mat2image(x);
    image sized = letterbox_image(im, net->w, net->h);
    layer output_layer = net->layers[net->n-1];
    
    /* Feed the precessed image to the network */
    float *X = sized.data;
    network_predict(net, X);

    /* Get the output of the network */
    int nboxes = 0;
    float thresh = 0.5;
    float hier_thresh = 0;
    detection *dets = get_network_boxes(net, im.w, im.h, thresh, hier_thresh, 0, 1, &nboxes);

    /* Do non-max suppresstion */
    float nms=.45;
    if (nms) do_nms_sort(dets, nboxes, output_layer.classes, nms);

    /* Save result to yoloObjects */
    for(int i = 0; i < nboxes; ++i){
        for(int j = 0; j < NUM_CLAS; ++j){
            if (dets[i].prob[j] > 0.5){
                    box b = dets[i].bbox;
                    int left  = (b.x-b.w/2.)*im.w;
                    int right = (b.x+b.w/2.)*im.w;
                    int top   = (b.y-b.h/2.)*im.h;
                    int bot   = (b.y+b.h/2.)*im.h;
                    YoloObject yoloObject;
                    yoloObject.bbox = Rect2d(left, top, right - left, bot - top);
                    yoloObject.objClass = j;
                    yoloObject.prob = dets[i].prob[j]*100;
                    yoloObject.name = string(names[j]);
                    yoloObjects.push_back(yoloObject);
            }
        }
    }
    free_image(im);
    free_image(sized);
    free_detections(dets, nboxes);
    return yoloObjects;
}