#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""断言一次决策巡视结果是否符合预期（读两份 CSV，不依赖 ROS）。

用法：
  /usr/bin/python3 scripts/check_decision_run.py \
      --results logs/results_xxx.csv --tasks logs/tasks_xxx.csv \
      --expect-order 4,3,2,1,5,6,7,8,12,11,10,9 \
      --expect-types 1,2,3,4,1,2,3,4,1,2,3,4 \
      --expect-handle 12 --expect-timeout 0

检查项：
  1) results CSV 恰好 12 行，visit_index 依次为 1..12
  2) 每个位点的 slot_id 等于 --expect-order
  3) material_type 等于 --expect-types（给了才查）
  4) tasks CSV 里 GOTO 恰好 12 条且位点顺序与 --expect-order 一致
  5) tasks CSV 里 HANDLE 条数 == --expect-handle
  6) results CSV 里 TIMEOUT 行数 == --expect-timeout
全部通过打印 PASS，否则逐条打印 FAIL 并以退出码 1 结束。
"""
import argparse
import csv
import sys


def read_rows(path):
    with open(path, newline='', encoding='utf-8') as fh:
        return list(csv.DictReader(fh))


def to_int_list(text):
    return [int(x) for x in text.split(',') if x.strip()]


def main():
    parser = argparse.ArgumentParser(description='校验决策巡视结果')
    parser.add_argument('--results', required=True, help='ResultLog 写出的结果 CSV')
    parser.add_argument('--tasks', required=True, help='record_task_topic.py 写出的任务 CSV')
    parser.add_argument('--expect-order', default='4,3,2,1,5,6,7,8,12,11,10,9',
                        help='12 个位点的 slot_id 期望顺序（默认：右下角出发蛇形）')
    parser.add_argument('--expect-types', default='',
                        help='12 个位点的物料类型期望值；留空则不校验')
    parser.add_argument('--expect-handle', type=int, default=12)
    parser.add_argument('--expect-timeout', type=int, default=0)
    parser.add_argument('--expect-goto', type=int, default=12)
    args = parser.parse_args()

    results = read_rows(args.results)
    tasks = read_rows(args.tasks)
    order = to_int_list(args.expect_order)
    types = to_int_list(args.expect_types) if args.expect_types else []

    failures = []

    def check(ok, label, detail=''):
        print('  [%s] %s%s' % ('PASS' if ok else 'FAIL', label, (' -> ' + detail) if detail else ''))
        if not ok:
            failures.append(label)

    print('检查结果 CSV：%s' % args.results)
    check(len(results) == len(order), '位点行数 = %d' % len(order), '实际 %d' % len(results))

    visits = [int(r['visit_index']) for r in results]
    check(visits == list(range(1, len(order) + 1)), 'visit_index 依次 1..N', str(visits))

    slots = [int(r['slot_id']) for r in results]
    check(slots == order, 'slot_id 顺序', '期望 %s / 实际 %s' % (order, slots))

    if types:
        got_types = [int(r['material_type']) for r in results]
        check(got_types == types, 'material_type 序列', '期望 %s / 实际 %s' % (types, got_types))

    timeouts = sum(1 for r in results if r['result'] == 'TIMEOUT')
    check(timeouts == args.expect_timeout, 'TIMEOUT 行数 = %d' % args.expect_timeout,
          '实际 %d' % timeouts)

    print('检查任务 CSV：%s' % args.tasks)
    gotos = [r for r in tasks if r['action'] == 'GOTO']
    handles = [r for r in tasks if r['action'] == 'HANDLE']
    check(len(gotos) == args.expect_goto, 'GOTO 条数 = %d' % args.expect_goto,
          '实际 %d' % len(gotos))
    check(len(handles) == args.expect_handle, 'HANDLE 条数 = %d' % args.expect_handle,
          '实际 %d' % len(handles))

    goto_slots = [int(r['slot_id']) for r in gotos]
    check(goto_slots == order[:len(goto_slots)], 'GOTO 位点顺序', '实际 %s' % goto_slots)
    goto_visits = [int(r['visit_index']) for r in gotos]
    check(goto_visits == list(range(1, len(gotos) + 1)), 'GOTO visit_index 依次 1..N',
          str(goto_visits))

    print('')
    if failures:
        print('结论：FAIL（%d 项不通过：%s）' % (len(failures), '; '.join(failures)))
        return 1
    print('结论：PASS（全部检查通过）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
