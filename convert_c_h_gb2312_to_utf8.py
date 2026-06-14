#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
递归将脚本所在目录及其子目录中的 .c / .h 文件，
从 GB2312/GBK/GB18030 转换为 UTF-8。

特点：
1. 只处理 .c 和 .h 文件
2. 递归处理子文件夹
3. 已经是 UTF-8 的文件会跳过
4. 默认会先备份原文件为 .bak
5. 控制台会输出处理结果
"""

from pathlib import Path
import sys

TARGET_SUFFIXES = {".c", ".h"}
SOURCE_ENCODINGS = ["gb2312", "gbk", "gb18030"]
BACKUP = True


def is_utf8(data: bytes) -> bool:
    try:
        data.decode("utf-8")
        return True
    except UnicodeDecodeError:
        return False


def try_decode_legacy(data: bytes):
    for enc in SOURCE_ENCODINGS:
        try:
            text = data.decode(enc)
            return text, enc
        except UnicodeDecodeError:
            continue
    return None, None


def convert_file(file_path: Path):
    try:
        raw = file_path.read_bytes()
    except Exception as e:
        return f"[读取失败] {file_path} -> {e}"

    if not raw:
        return f"[跳过空文件] {file_path}"

    if is_utf8(raw):
        return f"[已是 UTF-8] {file_path}"

    text, used_enc = try_decode_legacy(raw)
    if text is None:
        return f"[无法识别编码] {file_path}"

    try:
        if BACKUP:
            backup_path = file_path.with_suffix(file_path.suffix + ".bak")
            if not backup_path.exists():
                backup_path.write_bytes(raw)

        file_path.write_text(text, encoding="utf-8", newline="")
        return f"[转换成功] {file_path} ({used_enc} -> utf-8)"
    except Exception as e:
        return f"[写入失败] {file_path} -> {e}"


def main():
    root = Path(__file__).resolve().parent
    print(f"根目录: {root}")
    print("开始扫描 .c / .h 文件...\n")

    files = [
        p for p in root.rglob("*")
        if p.is_file() and p.suffix.lower() in TARGET_SUFFIXES
    ]

    if not files:
        print("未找到 .c 或 .h 文件。")
        return

    converted = 0
    skipped_utf8 = 0
    unknown = 0
    failed = 0

    for f in files:
        msg = convert_file(f)
        print(msg)

        if msg.startswith("[转换成功]"):
            converted += 1
        elif msg.startswith("[已是 UTF-8]") or msg.startswith("[跳过空文件]"):
            skipped_utf8 += 1
        elif msg.startswith("[无法识别编码]"):
            unknown += 1
        else:
            failed += 1

    print("\n处理完成：")
    print(f"  转换成功: {converted}")
    print(f"  跳过/已是 UTF-8: {skipped_utf8}")
    print(f"  无法识别编码: {unknown}")
    print(f"  失败: {failed}")

    if BACKUP:
        print("\n原文件备份为同名 .bak 文件。")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n用户中断。")
        sys.exit(1)
