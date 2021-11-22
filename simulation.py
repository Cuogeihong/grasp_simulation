import os
import multiprocessing
from main import sim
import random

pose_path = 'text_960'
result_path = 'text_960_res'
process_num = 10


def func(cmd):
    if sim(cmd[0], cmd[1]):
        return True
    return False
    #     success += 1
    # if random.randint(0, 5) < 3:
    #     success += 1
    # return random.randint(0, 5) < 3


def main():
    files = os.listdir(pose_path)
    for file in files:
        with open(os.path.join(pose_path, file), "r") as f:
            text_lines = f.readlines()
            # print((text_lines))
            for i in range(960):
                result = []
                cmd = file.split('.')[0], i
                pool = multiprocessing.Pool(processes=process_num)
                for j in range(10):
                    result.append(pool.apply_async(func, (cmd,)))
                    # result.append(func(cmd))
                pool.close()
                pool.join()
                print(cmd)
                success = 0
                for each in result:
                    if each.get():
                        # if each:
                        success += 1
                result_file = os.path.join(result_path, file)
                with open(result_file, 'a') as fo:
                    fo.write(text_lines[i * 6])
                    fo.write(text_lines[i * 6 + 1])
                    fo.write(text_lines[i * 6 + 2])
                    fo.write(text_lines[i * 6 + 3])
                    fo.write(text_lines[i * 6 + 4])
                    fo.write(str(success / 10))
                    fo.write(text_lines[i * 6 + 5])


if __name__ == '__main__':
    main()
