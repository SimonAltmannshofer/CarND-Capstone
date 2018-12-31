# From the tensorflow/models/research/ directory
PIPELINE_CONFIG_PATH=models/ssdv2tl_srb_v1_3/pipeline.config
MODEL_DIR=models/ssdv2tl_srb_v1_3/train_0
NUM_TRAIN_STEPS=10000
SAMPLE_1_OF_N_EVAL_EXAMPLES=1
python object_detection/train.py \
    --pipeline_config_path=${PIPELINE_CONFIG_PATH} \
    --train_dir=${MODEL_DIR} \
    --num_train_steps=${NUM_TRAIN_STEPS} \
    --sample_1_of_n_eval_examples=$SAMPLE_1_OF_N_EVAL_EXAMPLES \
    --alsologtostderr
