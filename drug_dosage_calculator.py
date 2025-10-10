#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
西罗莫司药物剂量计算器

根据体表面积计算西罗莫司的标准治疗剂量
标准治疗剂量：每平方米体表面积0.8mg，每日2次，间隔12小时服用
"""

import math


def calculate_bsa(weight_kg: float, height_cm: float) -> float:
    """
    计算体表面积（Body Surface Area）

    Args:
        weight_kg: 体重（公斤）
        height_cm: 身高（厘米）

    Returns:
        体表面积（平方米）
    """
    # BSA计算公式：体重^0.425 × 身高^0.725 × 0.007184
    bsa = (weight_kg ** 0.425) * (height_cm ** 0.725) * 0.007184
    return round(bsa, 3)  # 保留3位小数


def calculate_sirolimus_dosage(weight_kg: float, height_cm: float) -> dict:
    """
    计算西罗莫司药物剂量

    Args:
        weight_kg: 体重（公斤）
        height_cm: 身高（厘米）

    Returns:
        包含各种剂量信息的字典
    """
    # 标准治疗剂量：每平方米体表面积0.8mg
    standard_dosage_per_m2 = 0.8  # mg/m²

    # 计算体表面积
    bsa = calculate_bsa(weight_kg, height_cm)

    # 计算单次剂量
    single_dosage = standard_dosage_per_m2 * bsa  # mg

    # 计算每日剂量（每日2次）
    daily_dosage = single_dosage * 2  # mg/天

    return {
        '体表面积_BSA_m2': bsa,
        '单次剂量_mg': round(single_dosage, 2),
        '每日剂量_mg_per_day': round(daily_dosage, 2),
        '用药频率': '每日2次，间隔12小时',
        '标准剂量': f'{standard_dosage_per_m2} mg/m²'
    }


def print_dosage_info(weight_kg: float, height_cm: float) -> None:
    """
    打印剂量计算结果

    Args:
        weight_kg: 体重（公斤）
        height_cm: 身高（厘米）
    """
    result = calculate_sirolimus_dosage(weight_kg, height_cm)

    print("西罗莫司剂量计算结果：")
    print("-" * 30)
    print(f"体重：{weight_kg} kg")
    print(f"身高：{height_cm} cm")
    print(f"体表面积：{result['体表面积_BSA_m2']} m²")
    print(f"单次剂量：{result['单次剂量_mg']} mg")
    print(f"每日剂量：{result['每日剂量_mg_per_day']} mg/天")
    print(f"用药频率：{result['用药频率']}")
    print(f"计算标准：{result['标准剂量']}")


# 示例用法
if __name__ == "__main__":
    # 2岁儿童示例（体重12kg，身高85cm）
    print("示例计算（2岁儿童）：")
    print_dosage_info(12, 85)

    print("\n" + "="*50 + "\n")

    # 可以添加更多示例
    print("其他示例：")
    print_dosage_info(15, 90)  # 另一个示例
    print_dosage_info(20, 100)  # 另一个示例
