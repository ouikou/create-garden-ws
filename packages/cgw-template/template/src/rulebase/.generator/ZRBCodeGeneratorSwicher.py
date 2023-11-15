# coding:utf_8
import json
import sys
import os

from ZRBDtGeneratorCMD import ZRBDtGenerator
from ZRBStmGeneratorCMD import ZRBStmGenerator

make_lists_file_content = '''\
cmake_minimum_required(VERSION 3.5)
project(rule_{rule_name})

### ZRB_Rule_{rule_name} ###
add_library({rule_name} SHARED  "${{RULE_ROOT_DIR}}/rule_{rule_name}/src/ZRB_Rule_{rule_name}.cpp")
target_link_libraries({rule_name}  ${{SHARED_LIBRARY_DIR}}/libRBLib.so)

ament_target_dependencies({rule_name}
rclcpp
)
'''


def create_make_lists_file(file_path, rule_name):
    with open(file_path, 'w+', encoding='UTF-8') as f:
        f.write(make_lists_file_content.format(rule_name=rule_name))


args = sys.argv
if 3 == len(args):
    rule_dir = args[1]
    rule_name = args[2]

    # ファイルパス取得
    src_path = os.path.join(rule_dir, 'src')
    make_list_path = os.path.join(rule_dir, 'CMakeLists.txt')
    model_json_path = os.path.join(rule_dir, 'json', f'{rule_name}.json')
    define_json_path = os.path.join(rule_dir, 'json', f'{rule_name}Define.json')

    # ファイルが存在するかどうかをチェック
    if not os.path.exists(model_json_path):
        print(f'"{model_json_path}" does not exist.')
        sys.exit(1)
    if not os.path.exists(define_json_path):
        print(f'"{define_json_path}" does not exist.')
        sys.exit(1)

    # モデルのチェック & コード生成（stm / dt）
    json_open = open(model_json_path, 'r', encoding='utf-8')
    model = json.load(json_open)
    try:
        if 'dt' in model:
            # ソースフォルダ作成
            if not os.path.exists(src_path):
                os.makedirs(src_path)
            # CMakeLists.txtファイルの作成
            create_make_lists_file(make_list_path, rule_name)

            # Jsonデータの読み込み
            json_open = open(define_json_path, 'r', encoding='utf-8')
            dt_define = json.load(json_open)
            # Dtよりソースコード生成
            generator = ZRBDtGenerator(model, dt_define, rule_name, src_path)
            generator.generation()
            sys.exit(0)
        elif 'stm' in model:
            # ソースフォルダ作成
            if not os.path.exists(src_path):
                os.makedirs(src_path)
            # CMakeLists.txtファイルの作成
            create_make_lists_file(make_list_path, rule_name)

            # Jsonデータの読み込み
            json_open = open(define_json_path, 'r', encoding='utf-8')
            dt_define = json.load(json_open)
            rule = json.load(json_open)
            # Stmよりソースコード生成
            generator = ZRBStmGenerator(model, dt_define, rule_name, src_path)
            generator.generation()
            sys.exit(0)
        else:
            print(f'"{model_json_path}" not an DT or STM file.')
            sys.exit(1)
    except Exception as e:
        print(e)
        print(f'"{rule_name}" code generation failed.')
elif 3 < len(args):
    # 引数の数が3より多い場合エラー
    print('Arguments are too long.')
elif 3 > len(args):
    # 引数の数が3より少ない場合エラー
    print('Arguments are too short.')
