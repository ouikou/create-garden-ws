# coding:utf_8
import copy
import re
import os


class ZRBDtGenerator():

    def __init__(self, dt, dt_define, dt_filename, path):
        self.__rule_table = []
        self.__dt = dt
        self.__dt_define = dt_define
        self.__dt_name = dt_filename
        self.__rule_names = []
        self.__headder = ''
        self.__definition = ''
        self.__rules = ''
        self.__constructors = ''
        self.__functions = ''
        self.__path = path

    def search_fact_in_syntax(self, fact, syntax):
        # 現状FACT区切り文字をスペース" "、括弧"()"、イコール"="、アンド"&"、オア"|"にしている
        return re.search('([ (|&!<>=])'+fact+'([ )|&!<>=])|^'+fact+'([ |&!<>=])|([ |&!<>=])' + fact+'$', syntax)

    def extract_fact_from_syntax(self, syntax):
        for fact in self.__dt_define:
            if self.search_fact_in_syntax(fact, syntax):
                return fact

    def get_dt_data(self, rules):
        for rule_num, rule in enumerate(rules):
            rule_dict = {}
            rule_dict['rule'] = 'RULE'+str(rule_num+1)
            for id_num, cond in enumerate(rule['condition']):
                rule_dict['id'] = rule_dict['rule'] + '_'+'1'+'_'+str(id_num+1)
                rule_dict['condition'] = cond
                self.__rule_table.append(copy.deepcopy(rule_dict))
            rule_dict.clear()
            rule_dict['rule'] = 'RULE'+str(rule_num+1)
            for id_num, act in enumerate(rule['action']):
                rule_dict['id'] = rule_dict['rule'] + '_'+'2'+'_'+str(id_num+1)
                rule_dict['action'] = act
                self.__rule_table.append(copy.deepcopy(rule_dict))
            self.__rule_names.append(copy.copy(rule_dict['rule']))

    def make_syntax(self):
        for var_syntax, var_val in self.__dt_define.items():
            var_type = var_val['type']
            var_name = var_val['token']
            for row in filter(lambda e: e.get(var_type), self.__rule_table):
                if ('condition' in row and self.search_fact_in_syntax(var_name, row['condition'])):
                    exp = copy.copy(row['condition'])
                    row['syntax'] = exp.replace(var_name, var_syntax)
                if ('action' in row and self.search_fact_in_syntax(var_name, row['action'])):
                    proc = copy.copy(row['action'])
                    row['syntax'] = proc.replace(var_name, var_syntax)
            for val_syntax, val_val in var_val['values'].items():
                val_name = val_val['token']
                for row in filter(lambda e: e.get(var_type), self.__rule_table):
                    if ('condition' in row and self.search_fact_in_syntax(val_name, row['condition'])):
                        exp = copy.copy(row['condition'])
                        row['syntax'] = exp.replace(val_name, val_syntax)
                    if ('action' in row and self.search_fact_in_syntax(val_name, row['action'])):
                        proc = copy.copy(row['action'])
                        row['syntax'] = proc.replace(val_name, val_syntax)

        # rule_tableにすでにsyntaxが登録されている場合
        for var_syntax, var_val in self.__dt_define.items():
            var_name = var_val['token']
            for row in self.__rule_table:
                if ('syntax' in row):
                    if (self.search_fact_in_syntax(var_name, row['syntax'])):
                        stx = copy.copy(row['syntax'])
                        row['syntax'] = stx.replace(var_name, var_syntax)
            for val_syntax, val_val in var_val['values'].items():
                val_name = val_val['token']
                for row in self.__rule_table:
                    if ('syntax' in row):
                        if (self.search_fact_in_syntax(val_name, row['syntax'])):
                            stx = copy.copy(row['syntax'])
                            row['syntax'] = stx.replace(val_name, val_syntax)

    def set_fact_to_cond_act(self):
        for row in self.__rule_table:
            for fact in self.__dt_define:
                if self.search_fact_in_syntax(fact, row['syntax']):
                    row['fact'] = fact

    def make_rule_table(self):
        for row in self.__rule_table:
            if ('condition' in row):
                # if関数名登録
                row['if_func'] = 'int32_t Rule_if_'+row['id']+'()'
                if ('syntax' not in row):
                    # ruleファイルに条件・処理がなく、決定表モデルに条件・処理が直接書かれていた場合の処理
                    row['syntax'] = row['condition']
                    row['fact'] = self.extract_fact_from_syntax(row['syntax'])
            elif ('action' in row):
                # then関数名登録
                row['then_func'] = 'void Rule_then_'+row['id']+'()'
                if ('syntax' not in row):
                    # ruleファイルに条件・処理がなく、決定表モデルに条件・処理が直接書かれていた場合の処理
                    row['syntax'] = row['action']
                    row['fact'] = self.extract_fact_from_syntax(row['syntax'])
            else:
                pass

    def make_CPP_headder(self):
        self.__headder = ('# ifndef RBRULE_H\n'
                          '# define RBRULE_H\n'
                          '# include "' + 'RBLib.h"\n\n'
                          '#include <rclcpp/rclcpp.hpp>\n'
                          '/////////////////////\n'
                          '// Fact Definition\n'
                          '/////////////////////\n'
                          )
        fact_val_list = []
        for fact_elem in self.__dt_define.items():
            for fact_key, fact_val in fact_elem[1]['values'].items():
                current_fact = (fact_key, fact_val['value'])
                if (current_fact not in fact_val_list):
                    # 同じFACTの値が複数FACTに登録される場合もあるので、FACT値を重複しないように一旦リストに保存
                    fact_val_list.append(current_fact)
        for val in fact_val_list:
            self.__headder = self.__headder + '#define ' + \
                val[0]+' '+str(val[1])+'\n'

        self.__headder = self.__headder + '\nenum e_FACT {\n'
        key_list = list(self.__dt_define.keys())
        self.__headder = self.__headder + ' ' + key_list[0] + ' = 0, \n'
        for key in key_list[1:]:
            self.__headder = self.__headder + ' ' + key + ',\n'
        self.__headder = self.__headder + ' FACT_END\n};\n'

        self.__headder = self.__headder + ('\n/////////////////////\n'
                                           '// Rule Definition\n'
                                           '/////////////////////\n'
                                           )
        self.__headder = self.__headder + 'enum e_Rule {\n'

        rule_names = list(set([d.get('rule') for d in self.__rule_table]))
        rule_names.sort()
        self.__headder = self.__headder + ' ' + rule_names[0] + ' = 0, \n'
        for rule_name in rule_names[1:]:
            self.__headder = self.__headder + ' ' + rule_name + ',\n'
        self.__headder = self.__headder + ' RULE_END\n};\n'

        self.__headder = self.__headder + ('\n/////////////////////////\n'
                                           '// RuleQue Definition\n'
                                           '/////////////////////////\n'
                                           )
        for key in self.__dt_define.keys():
            self.__headder = self.__headder + '#define '+'RULE_QUE_MAX_' + key+' ' + \
                str(len(set([d.get('rule') for d in filter(
                    lambda item: item['fact'] == key, self.__rule_table)])))+'\n'

        self.__headder = self.__headder + ('\n/////////////////////////\n'
                                           '// Function\n'
                                           '/////////////////////////\n'
                                           'extern void ZRB_RuleInitialize();\n'
                                           'extern fact_info_s* ZRB_GetFactPtr();\n'
                                           'extern rule_info_s* ZRB_GetRulePtr();\n'
                                           'extern void ZRB_SetPointer(rclcpp::Node* nodePointer);\n'
                                           '\n#endif\n')

    def make_CPP_code(self):
        self.__definition = ('#include <stdio.h>\n'
                             '#include <stdint.h>\n'
                             '#include "ZRB_Rule_' +
                             f'{self.__dt_name}.h'+'"\n\n'
                             '//////////////////////////\n'
                             '// Definition\n'
                             '//////////////////////////\n\n'
                             'rclcpp::Node* nPointer;\n'
                             'fact_info_s fact_info;\n'
                             'rule_info_s rule_info;\n'
                             'fact_s fact[FACT_END];\n'
                             'rule_s rule[RULE_END];\n'
                             )
        rule_func_list = []
        for rule_name in self.__rule_names:
            # ルール名でrule_tableをフィルタしてif_func,then_funcがあるレコードを重複を削除して検索し、rule_func_listに登録
            if_funcs = set([d.get('if_func') for d in filter(
                lambda item: item['rule'] == rule_name, self.__rule_table) if d.get('if_func') is not None])
            then_funcs = set([d.get('then_func') for d in filter(
                lambda item: item['rule'] == rule_name, self.__rule_table) if d.get('then_func') is not None])
            rule_func_list.append(
                dict(rule=rule_name, if_func=if_funcs, then_func=then_funcs))
            # if_func,then_funcがあるレコードのヒット数をカウントしてRule_if, Rule_thenテーブルの要素数として設定
            self.__definition = self.__definition + 'rule_if Rule_if_' + \
                rule_name + '[' + str(len(if_funcs))+'];\n'
            self.__definition = self.__definition + 'rule_then Rule_then_' + \
                rule_name + '[' + str(len(then_funcs))+'];\n'
        print("<< rule_func_list >>")
        # print(rule_func_list)

        for key in self.__dt_define.keys():
            self.__definition = self.__definition + \
                'rule_que rule_que_' + key+'[RULE_QUE_MAX_'+key+'];\n'

        self.__rules = ('//////////////////////////\n'
                        '// Rules\n'
                        '//////////////////////////\n')
        if_func_fact_list = []
        if_func_list = set(
            [d.get('if_func') for d in self.__rule_table if d.get('if_func') is not None])
        for if_func in if_func_list:
            rows = list(filter(lambda item: item.get(
                'if_func') == if_func, self.__rule_table))
            fact_name = list(set([d.get('fact') for d in rows]))
            syntax = list(set([d.get('syntax') for d in rows]))[0]
            if_func_fact_list.append(dict(if_func=if_func, fact=fact_name, syntax=syntax))
        print("<< if_func_list >>")
        # print(if_func_list)
        for if_func in if_func_fact_list:
            self.__rules = self.__rules + \
                '\n'+if_func['if_func']+'\n{\n'
            self.__rules = self.__rules + \
                ' RCLCPP_INFO(nPointer->get_logger(),"' + \
                if_func['if_func'] + '\\n");\n'
            fact_num = ''
            for num, fact in enumerate(if_func['fact']):
                # 複数FACTの場合はfact_p1,2,3...と番号を振って変数を区別する。1つめのみ番号なし
                self.__rules = self.__rules + ' fact_s *fact_p' + \
                    fact_num + ' = &fact[' + fact + '];\n'
                if_func['syntax'] = re.sub('([ (|&!<>=])'+fact+'([ )|&!<>=])|^'+fact + '([ |&!<>=])|([ |&!<>=])' +
                                           fact+'$', '\\1\\4fact_p'+fact_num+'->fact_value\\2\\3', if_func['syntax'])
                fact_num = str(num+1)
            self.__rules = self.__rules + \
                ' if(' + if_func['syntax'].lstrip(' ') + ')\n'
            self.__rules = self.__rules + (
                ' {\n'
                '   RCLCPP_INFO(nPointer->get_logger(),"TRUE\\n");\n'
                '   return TRUE;\n'
                ' }\n'
                ' else\n'
                ' {\n'
                '   RCLCPP_INFO(nPointer->get_logger(),"FALSE\\n");\n'
                '   return FALSE;\n'
                ' }\n'
                '}\n'
            )

        then_func_fact_list = []
        then_func_list = set(
            [d.get('then_func') for d in self.__rule_table if d.get('then_func') is not None])
        for then_func in then_func_list:
            rows = list(filter(lambda item: item.get(
                'then_func') == then_func, self.__rule_table))
            fact_name = list(set([d.get('fact') for d in rows]))
            syntax = list(set([d.get('syntax') for d in rows]))
            then_func_fact_list.append(
                dict(then_func=then_func, fact=fact_name, syntax=syntax))
        print("<< then_func_list >>")
        # print(then_func_list)
        for then_func in then_func_fact_list:
            self.__rules = self.__rules + \
                '\n'+then_func['then_func']+'\n{\n'
            self.__rules = self.__rules + \
                ' RCLCPP_INFO(nPointer->get_logger(),"' + \
                then_func['then_func'] + '\\n");\n'
            for fact in then_func['fact']:
                for syntax in then_func['syntax']:
                    if (self.search_fact_in_syntax(fact, syntax)):
                        syntax = re.sub('([ (|&!<>=])'+fact+'([ )|&!<>=])|^'+fact + '([ |&!<>=])|([ |&!<>=])' +
                                        fact+'$', '\\1\\4 fact[' + fact+'].fact_value\\2\\3', syntax)
                        self.__rules = self.__rules + \
                            syntax + ';\n'
                        break
            self.__rules = self.__rules + '}\n'

        self.__constructors = self.__constructors + ('\nvoid construct_rule()\n'
                                                     '{\n'
                                                     ' rule_if *rule_if_p;\n'
                                                     ' rule_then *rule_then_p;\n'
                                                     ' rule_s *rule_p;\n')
        for row in rule_func_list:
            rule_name = row.get('rule')
            self.__constructors = self.__constructors + \
                '\n rule_if_p = Rule_if_' + rule_name + ';\n'
            if_cnt = 0
            for if_cnt, if_func in enumerate(row.get('if_func')):
                self.__constructors = self.__constructors + (' rule_if_p['+str(if_cnt)+'].IF_ptr = &'+if_func.replace('int32_t ', '').replace('(', '').replace(')', '')+';\n'
                                                             ' rule_if_p['+str(if_cnt)+'].result = FALSE;\n')
                rule_table_row = list(filter(lambda item: item.get(
                    'rule') == rule_name and item.get('if_func') == if_func, self.__rule_table))
                rule_table_row[0]['if_index'] = str(if_cnt)
            self.__constructors = self.__constructors + \
                ' rule_then_p = Rule_then_' + rule_name + ';\n'
            then_cnt = 0
            for then_cnt, then_func in enumerate(row.get('then_func')):
                self.__constructors = self.__constructors + \
                    ' rule_then_p['+str(then_cnt)+'].THEN_ptr = &' + \
                    then_func.replace('void ', '').replace(
                        '(', '').replace(')', '')+';\n'
            self.__constructors = self.__constructors + (' rule_p = &rule['+rule_name+'];\n'
                                                         ' rule_p->rule_ID = '+rule_name+';\n')
            self.__constructors = self.__constructors + (' rule_p->rule_IF_max = '+str(if_cnt+1) + ';\n'
                                                         ' rule_p->rule_IF_first_ptr = rule_if_p;\n'
                                                         ' rule_p->rule_THEN_max = ' +
                                                         str(then_cnt+1) +
                                                         ';\n'
                                                         ' rule_p->rule_THEN_first_ptr = rule_then_p;\n')
            self.__constructors = self.__constructors + (' rule_p->agenda_previous_ptr = NULL;\n'
                                                         ' rule_p->agenda_next_ptr = NULL;\n'
                                                         ' rule_p->fired_previous_ptr = NULL;\n'
                                                         ' rule_p->fired_next_ptr = NULL;\n')
        self.__constructors = self.__constructors + '}\n'

        self.__constructors = self.__constructors + ('\nvoid construct_ruleQue()\n'
                                                     '{\n'
                                                     ' rule_que *rule_que_p;\n\n')
        for key in self.__dt_define.keys():
            # rule_tableを対象FACTでif関数があるレコードに絞り込む
            fact_rows = list(filter(
                lambda item: item['fact'] == key and 'if_func' in item.keys(), self.__rule_table))
            if (fact_rows != []):
                # if関数のあるFACTのみrule_queに出力する
                self.__constructors = (self.__constructors + ' //'+key+'\n'
                                       ' rule_que_p = rule_que_' + key+';\n')
                for cnt, row in enumerate(fact_rows):
                    rq_head = ' rule_que_p[' + str(cnt) + '].'
                    self.__constructors = self.__constructors + \
                        rq_head+'rule_ID_no = '+row['rule']+';\n'
                    self.__constructors = self.__constructors + \
                        (rq_head+'rule_IF_no = '+row['if_index']+';\n'
                         + rq_head+'rule_group_no = 0;\n'
                         + rq_head+'rule_priority = 0;\n')
                    if (cnt == len(fact_rows)-1):
                        self.__constructors = self.__constructors + rq_head+'RQ_next_ptr = NULL;\n'
                    else:
                        self.__constructors = self.__constructors + rq_head + \
                            'RQ_next_ptr = &rule_que_p['+str(cnt+1)+'];\n'
                    if (cnt == 0):
                        self.__constructors = self.__constructors + rq_head+'RQ_previous_ptr = NULL;\n'
                    else:
                        self.__constructors = self.__constructors + rq_head + \
                            'RQ_previous_ptr = &rule_que_p['+str(cnt-1)+'];\n'
        self.__constructors = self.__constructors + '}\n'
        self.__constructors = self.__constructors + ('void initial_fact_common_info(fact_s *fact_p)\n'
                                                     '{\n'
                                                     ' fact_p->Rule_Que_current_no = 0;\n'
                                                     ' fact_p->fact_status = READY;\n'
                                                     ' fact_p->fact_value = -1;\n'
                                                     ' fact_p->fact_priority = 0;\n'
                                                     ' fact_p->interval_timer = 0;\n'
                                                     ' fact_p->fact_History_max = 0;\n'
                                                     ' fact_p->fact_History_current_no = 0;\n'
                                                     ' fact_p->fact_History_first_ptr = NULL;\n'
                                                     ' fact_p->time_support = 0;\n'
                                                     ' fact_p->fact_time = 0;\n'
                                                     ' fact_p->fact_time_History_first_ptr = NULL;\n'
                                                     '}\n')

        self.__constructors = self.__constructors + ('\nvoid construct_fact()\n'
                                                     '{\n'
                                                     ' fact_s *fact_p;\n')
        for key in self.__dt_define.keys():
            self.__constructors = self.__constructors + \
                (' fact_p = &fact['+key+'];\n'
                 + ' fact_p->fact_ID = '+key+';\n'
                 + ' fact_p->Rule_Que_Max = RULE_QUE_MAX_'+key+';\n'
                 + ' fact_p->Rule_Que_first_ptr = rule_que_'+key+';\n'
                 + ' fact_p->Rule_Que_last_ptr = &rule_que_' +
                 key+'[RULE_QUE_MAX_'+key+'- 1];\n'
                 + ' initial_fact_common_info(fact_p);\n')
        self.__constructors = self.__constructors + '}\n'

        self.__functions = self.__functions + ('\nvoid ZRB_RuleInitialize()\n{\n'
                                               ' RCLCPP_INFO(nPointer->get_logger(),"ZRB_RuleInitialize()\\n");\n'
                                               ' construct_rule();\n'
                                               ' construct_fact();\n'
                                               ' construct_ruleQue();\n'
                                               '}\n')
        self.__functions = self.__functions + ('\nfact_info_s* ZRB_GetFactPtr()\n{\n'
                                               ' RCLCPP_INFO(nPointer->get_logger(),"ZRB_GetFactPtr() return[%p]\\n", &fact[0]);\n'
                                               ' fact_info.fact_max = FACT_END;\n'
                                               ' fact_info.fact_ptr = fact;\n'
                                               ' return &fact_info;\n'
                                               '}\n')
        self.__functions = self.__functions + ('\nrule_info_s* ZRB_GetRulePtr()\n{\n'
                                               ' RCLCPP_INFO(nPointer->get_logger(),"ZRB_GetRulePtr() return[%p]\\n", &rule[0]);\n'
                                               ' rule_info.rule_group_max = 0;\n'
                                               ' rule_info.rule_group_p = 0;\n'
                                               ' rule_info.rule_max = RULE_END;\n'
                                               ' rule_info.rule_ptr = rule;\n'
                                               ' return &rule_info;\n'
                                               '}\n')
        self.__functions = self.__functions + ('\nvoid ZRB_SetPointer(rclcpp::Node* nodePointer)\n{\n'
                                               ' nPointer = nodePointer;\n'
                                               '}\n')

    def generation(self):
        self.get_dt_data(self.__dt['dt']['rules'])
        self.make_syntax()
        self.set_fact_to_cond_act()
        self.make_rule_table()
        self.make_CPP_headder()
        self.make_CPP_code()

        cpp_str = self.__definition + self.__rules + \
            self.__constructors + self.__functions
        headder = open(os.path.join(self.__path, f'ZRB_Rule_{self.__dt_name}.h'), 'w+', encoding='UTF-8')
        headder.write(self.__headder)
        headder.close()
        cpp = open(os.path.join(self.__path, f'ZRB_Rule_{self.__dt_name}.cpp'), 'w+', encoding='UTF-8')
        cpp.write(cpp_str)
        cpp.close()
