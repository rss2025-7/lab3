import importlib
import rosbag2_py
import rclpy.serialization

# --- Configuration parameters ---
BAG_DIRECTORY = './rosbag2_2025_03_07-21_16_07'
DESIRED_DISTANCE = 0.0 # really the desired error
ALPHA = 1         # The alpha parameter used in the score function
V = 1.5

def deserialize_message(topic, data):
    topic_type_map = {
        '/wall_follower_logs': 'std_msgs.msg.Float32',
        # Add other topic mappings as needed.
    }

    if topic not in topic_type_map:
        print(f"No message type mapping for topic: {topic}")
        return None

    msg_type_str = topic_type_map[topic]
    try:
        module_name, class_name = msg_type_str.rsplit('.', 1)
        mod = importlib.import_module(module_name)
        msg_class = getattr(mod, class_name)
    except Exception as e:
        print(f"Error importing message class for topic {topic}: {e}")
        return None

    try:
        msg = rclpy.serialization.deserialize_message(data, msg_class)
        return msg
    except Exception as e:
        print(f"Error deserializing message on topic {topic}: {e}")
        return None

# --- Scoring function ---
def compute_score(bag_path, desired_distance, alpha):
    """
    Reads a ROS2 bag file from the given directory, computes the average loss
    (mean absolute error of the distance values), and then calculates a score.
    
    The loss is defined as:
        loss = (1/N) * sum(|distance_i - desired_distance|)
        
    The score is:
        score = 1 / (1 + (alpha * loss)^2)
    """
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    distances = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        msg = deserialize_message(topic, data)
        if msg is not None:
            try:
                # For std_msgs.msg.Float32, the float value is stored in 'data'
                distances.append(msg.data)
            except AttributeError:
                print(f"Message on topic {topic} does not have a 'data' attribute.")

    if not distances:
        print("No valid messages found in the bag file.")
        return None

    N = len(distances)
    total_loss = sum(abs(d - desired_distance) for d in distances)
    avg_loss = total_loss / N

    score = 1 / (1 + (alpha * avg_loss)**2)

    print(f"Processed {N} messages.")
    print(f"Average Loss: {avg_loss}")
    print(f"Score: {score}")
    return score

# --- Main execution ---
if __name__ == '__main__':
    compute_score(BAG_DIRECTORY, DESIRED_DISTANCE, ALPHA)
