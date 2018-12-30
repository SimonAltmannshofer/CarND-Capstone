# From the tensorflow/models/research/ directory
PIPELINE_CONFIG_PATH=models/ssdv2tl_srb_V1_3/pipeline.config
TRAIN_DIR=models/ssdv2tl_srb_V1_3/train_1
NUM_TRAIN_STEPS=10000
SAMPLE_1_OF_N_EVAL_EXAMPLES=1
python object_detection/train.py \
    --pipeline_config_path=${PIPELINE_CONFIG_PATH} \
    --train_dir=${TRAIN_DIR}
#    --num_train_steps=${NUM_TRAIN_STEPS} \
#    --sample_1_of_n_eval_examples=$SAMPLE_1_OF_N_EVAL_EXAMPLES \
#    --alsologtostderr
