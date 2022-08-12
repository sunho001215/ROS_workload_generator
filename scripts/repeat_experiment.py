import os
import signal
import psutil
import rospkg
from tqdm import trange
from time import sleep
from response_time import read_file, calculate_response_time, get_average_response_time, get_max_response_time

########### PARAM ###########
iter_num_ = 100
test_cmd_ = "taskset -c 11 roslaunch ros_workload_generator simple_chain.launch"
test_exec_time_ = 30
#############################

rospack = rospkg.RosPack()
pkg_path = rospack.get_path("ros_workload_generator")

def kill_via_pid(pid):
    os.kill(pid, signal.SIGTERM)

def kill_all_ros_node():
    for proc in psutil.process_iter():
        processName = proc.name()
        processID = proc.pid
        if "ros" in processName or "node" in processName:
            os.kill(processID, signal.SIGTERM)

def ros_fork_execute():
    pid = os.fork()
    if pid == 0:
        os.system(test_cmd_ + " > /dev/null")
        return
    
    sleep(test_exec_time_)
    kill_via_pid(pid)
    kill_all_ros_node()
    sleep(0.1)
    
def calc_response_time():
    entry_file_path = pkg_path + "/csv/entry_node.csv"
    leaf_file_path = pkg_path + "/csv/leaf_node.csv"
    start_idx_list, end_idx_list, start_time_list, end_time_list = read_file(entry_file_path, leaf_file_path)
    response_time_list = calculate_response_time(start_idx_list, end_idx_list, start_time_list, end_time_list)

    max_response_time = get_max_response_time(response_time_list)
    avg_response_time = get_average_response_time(response_time_list)

    return max_response_time, avg_response_time

def write_txt_file(max_response_time_list, avg_response_time_list):
    max_txt_path = pkg_path + "/max_response_time.txt"
    avg_txt_path = pkg_path + "/avg_response_time.txt"

    with open(max_txt_path, 'w+') as f:
        f.write('\n'.join(max_response_time_list))

    with open(avg_txt_path, 'w+') as f:
        f.write('\n'.join(avg_response_time_list))

if __name__ == "__main__":
    max_response_time_list, avg_response_time_list = [], []

    for i in trange(iter_num_):
        ros_fork_execute()
        max_response_time, avg_response_time = calc_response_time()
        max_response_time_list.append(str(max_response_time))
        avg_response_time_list.append(str(avg_response_time))
    
    write_txt_file(max_response_time_list, avg_response_time_list)
