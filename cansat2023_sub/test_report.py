import os

def get_last_file_in_folder(folder_path):
    # フォルダ内のファイル一覧を取得し、最後のファイルを選択
    file_list = os.listdir(folder_path)
    if file_list:
        file_list.sort()  # ファイル名でソート
        last_file = os.path.join(folder_path, file_list[-1])
        print(last_file)
        return last_file

    else:
        return "フォルダ内にファイルが見つかりませんでした。"

def delete_lines_below(file_path, line_number):
    # ファイルの指定した行以下を削除する
    try:
        with open(file_path, 'r') as file:
            lines = file.readlines()

        with open(file_path, 'w') as file:
            for i, line in enumerate(lines, 1):
                if i > line_number:
                    continue
                file.write(line)

        print(f"ファイル {file_path} の {line_number} 行以下をすべて消去しました。")
    except FileNotFoundError:
        print(f"ファイル {file_path} が見つかりません。")


def extract_and_write(input_file, output_file, start_line, end_line=float('inf')):
    # ファイルの指定した範囲を取得し、別のファイルに追記する
    try:
        with open(input_file, 'r') as infile:
            lines = infile.readlines()
            start_line_index = max(0, start_line - 1)
            end_line_index = min(len(lines), end_line)

            extracted_lines = lines[start_line_index:end_line_index]

        # ファイルの内容を追記モードで開き、元の内容を保持したまま新しい内容を追記
        with open(output_file, 'a') as outfile:
            outfile.writelines(extracted_lines)

        print(f"{len(extracted_lines)}行が {output_file} に追記されました。")
    except Exception as e:
        print(f"エラーが発生しました: {e}")

def extract_single_line(input_file, output_file, line_number):
    # ファイルの指定した1行を取得し、別のファイルに追記する
    try:
        with open(input_file, 'r') as infile:
            lines = infile.readlines()

        # 指定した行を抽出
        line_to_extract = lines[line_number - 1]

        # ファイルの内容を追記モードで開き、元の内容を保持したまま新しい内容を追記
        with open(output_file, 'a') as outfile:
            outfile.write(line_to_extract)

        print(f"{line_number}行目が {output_file} に書き込まれました。")
    except Exception as e:
        print(f"エラーが発生しました: {e}")

def add_text_to_last_line(output_file, text_to_add):
    # ファイルにテキストを追加する
    try:
        with open(output_file, 'a') as outfile:
            # 「あいうえお」を追加して新しいファイルに書き込む
            outfile.write(text_to_add + '\n')

        print(f"「{text_to_add}」が {output_file} の最下行に追加されました。")
    except Exception as e:
        print(f"エラーが発生しました: {e}")

#report file
output_file = './control_record_report.txt'
input_file = get_last_file_in_folder('./logs/0_report_log')

#reset
line_to_delete_below = 7
delete_lines_below(output_file, line_to_delete_below)
print("0")

#開始時間
text_to_add = "Time and position at which control was initiated:"
add_text_to_last_line(output_file, text_to_add)
print("1")

# input_file = get_last_file_in_folder('./logs/0_report_log')
line_number=2
extract_single_line(input_file, output_file, line_number)
print("2")

#終了時間
text_to_add = "Time and position at which the control ended:"
add_text_to_last_line(output_file, text_to_add)
print("3")

line_number=-1
extract_single_line(input_file, output_file, line_number)
print("4")

text_to_add = "All control history:"
add_text_to_last_line(output_file, text_to_add)
print("4")

#report_log
# input_file = get_last_file_in_folder('./logs/0_report_log')
start_line = 1
extract_and_write(input_file, output_file, start_line)
print("5")

# #log_phase
# input_file = get_last_file_in_folder('./log/phaselog')
# start_line = 1
# extract_and_write(input_file, output_file, start_line)
# print("5")

# text_to_add = "Detailed control record:"
# add_text_to_last_line(output_file, text_to_add)
# print("6")

# #log_release
# input_file = get_last_file_in_folder('./log/releaselog')
# start_line = 1
# extract_and_write(input_file, output_file, start_line)
# print("7")

# #log_landing
# input_file = get_last_file_in_folder('./log/landinglog')
# start_line = 1
# extract_and_write(input_file, output_file, start_line)
# print("8")

# #log_melting
# input_file = get_last_file_in_folder('./log/meltinglog')
# start_line = 1
# extract_and_write(input_file, output_file, start_line)
# print("9")

# #log_para
# input_file = get_last_file_in_folder('./log/para_avoid_log')
# start_line = 1
# extract_and_write(input_file, output_file, start_line)
# print("10")

# #log_gpsrunning1
# input_file = get_last_file_in_folder('./log/gpsrunning1log')
# start_line = 1
# extract_and_write(input_file, output_file, start_line)
# print("11")

# #log_humandetect
# input_file = get_last_file_in_folder('./log/humandetectlog')
# start_line = 1
# extract_and_write(input_file, output_file, start_line)
# print("12")

# #log_gpsrunning2
# input_file = get_last_file_in_folder('./log/gpsrunning2log')
# start_line = 1
# extract_and_write(input_file, output_file, start_line)
# print("13")

# #log_photorunning
# input_file = get_last_file_in_folder('./log/photorunninglog')
# start_line = 1
# extract_and_write(input_file, output_file, start_line)
# print("14")