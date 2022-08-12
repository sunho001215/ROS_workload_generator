import rospkg
import matplotlib.pyplot as plt
import math

def read_file(entry_file_path, leaf_file_path):
    start_idx_list, end_idx_list, start_time_list, end_time_list = [], [], [], []

    with open(entry_file_path, 'r') as f:
        line = f.readline()
        line = f.readline()
        while line:
            line_split = line.split(",")
            start_idx = int(line_split[4])
            start_time = float(line_split[2])
            
            start_idx_list.append(start_idx)
            start_time_list.append(start_time)

            line = f.readline()

    with open(leaf_file_path, 'r') as f:
        line = f.readline()
        line = f.readline()
        while line:
            line_split = line.split(",")
            end_idx = int(line_split[4])
            end_time = float(line_split[3])
            
            end_idx_list.append(end_idx)
            end_time_list.append(end_time)

            line = f.readline()
    
    return start_idx_list, end_idx_list, start_time_list, end_time_list

def calculate_response_time(start_idx_list, end_idx_list, start_time_list, end_time_list):
    response_time_list = []

    start_idx_len = len(start_idx_list)
    end_idx_len = len(end_idx_list)

    start_idx, end_idx = 0, 0
    while True:
        if not start_idx < start_idx_len:
            break
        if not end_idx < end_idx_len:
            break

        if start_idx_list[start_idx] == end_idx_list[end_idx]:
            response_time_list.append(end_time_list[end_idx] - start_time_list[start_idx])
            start_idx = start_idx + 1
            end_idx = end_idx + 1
        elif start_idx_list[start_idx] > end_idx_list[end_idx]:
            end_idx = end_idx + 1
        else:
            start_idx = start_idx + 1
            
    return response_time_list

def get_average_response_time(response_time):
    sum = 0
    for value in response_time:
        sum += value
    return sum / len(response_time)

def get_max_response_time(response_time):
    max = 0
    for value in response_time:
        if(value > max):
            max = value
    return max

def get_min_response_time(response_time):
    min = 999999
    for value in response_time:
        if(value < min):
            min = value
    return min

def get_var_response_time(response_time):
    avg = sum(response_time) / len(response_time)
    var = sum((x-avg)**2 for x in response_time) / len(response_time)
    return math.sqrt(var)

if __name__ == "__main__":
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("ros_workload_generator")

    entry_file_path = pkg_path + "/csv/entry_node.csv"
    leaf_file_path = pkg_path + "/csv/leaf_node.csv"
    png_path = pkg_path + "/response_time.png"

    start_idx_list, end_idx_list, start_time_list, end_time_list = read_file(entry_file_path, leaf_file_path)
    response_time_list = calculate_response_time(start_idx_list, end_idx_list, start_time_list, end_time_list)

    print(get_min_response_time(response_time_list))
    print(get_max_response_time(response_time_list))
    print(get_average_response_time(response_time_list))
    print(get_var_response_time(response_time_list))
    
    # print(data)
    plt.ylim(0,0.9)
    plt.xlim(0,len(response_time_list)-1)

    # plt.axhline(y=400, color='r', linewidth=1, label='Deadline: 400ms')

    plt.plot(response_time_list, linewidth=0.5 ,color="black", label='Execution Time')
    plt.xlabel("Instance Number")
    plt.ylabel("Response Time (ms)")
    
    plt.savefig(png_path)
    plt.close()
    
