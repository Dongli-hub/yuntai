"""极简 YAML 子集解析器 / 生成器。

存在的理由：地瓜派（RDK）上未必装了 PyYAML，而配置必须能读。
本模块只覆盖配置文件真正用到的语法子集：

    支持：
      key: value                 标量
      key:                       嵌套映射
        sub: value
      key:                       块式列表
        - a
        - b
      key: [a, b, c]             行内列表
      # 注释、空行、单/双引号、int/float/bool/null

    不支持（配置里也不要用）：
      锚点 & 别名、多行字符串 |/>、复杂嵌套的行内映射、tag、文档分隔符 ---

若环境里装了 PyYAML，`config.py` 会优先用它；本模块是纯回退。
两者对本配置用到的子集应当完全一致（tests 里有往返测试）。
"""

from typing import Any, List, Tuple

__all__ = ["loads", "dumps", "load", "dump"]


def _strip_comment(line: str) -> str:
    """去掉引号外的 # 注释，并去掉行尾空白。"""
    out = []
    quote = None
    for ch in line:
        if quote is not None:
            out.append(ch)
            if ch == quote:
                quote = None
        elif ch in "\"'":
            quote = ch
            out.append(ch)
        elif ch == "#":
            break
        else:
            out.append(ch)
    return "".join(out).rstrip()


def _split_top(text: str, sep: str = ",") -> List[str]:
    """按分隔符切分，忽略引号/括号内部的分隔符。"""
    parts, buf, depth, quote = [], [], 0, None
    for ch in text:
        if quote is not None:
            buf.append(ch)
            if ch == quote:
                quote = None
        elif ch in "\"'":
            quote = ch
            buf.append(ch)
        elif ch in "[{(":
            depth += 1
            buf.append(ch)
        elif ch in "]})":
            depth -= 1
            buf.append(ch)
        elif ch == sep and depth == 0:
            parts.append("".join(buf))
            buf = []
        else:
            buf.append(ch)
    if buf:
        parts.append("".join(buf))
    return parts


def _scalar(token: str) -> Any:
    t = token.strip()
    if t == "":
        return None
    if len(t) >= 2 and t[0] == t[-1] and t[0] in "\"'":
        return t[1:-1]
    low = t.lower()
    if low in ("null", "~", "none"):
        return None
    if low in ("true", "yes", "on"):
        return True
    if low in ("false", "no", "off"):
        return False
    if t.startswith("[") and t.endswith("]"):
        inner = t[1:-1].strip()
        if not inner:
            return []
        return [_scalar(p) for p in _split_top(inner)]
    if t.startswith("{") and t.endswith("}"):
        inner = t[1:-1].strip()
        result = {}
        if inner:
            for item in _split_top(inner):
                key, _, val = item.partition(":")
                result[key.strip().strip("\"'")] = _scalar(val)
        return result
    for cast in (int, float):
        try:
            return cast(t)
        except ValueError:
            pass
    return t


def _prepare(text: str) -> List[Tuple[int, str]]:
    lines: List[Tuple[int, str]] = []
    for raw in text.splitlines():
        lead = raw[: len(raw) - len(raw.lstrip(" \t"))]
        if "\t" in lead:
            raise ValueError("YAML 缩进不允许用 Tab，请改用空格")
        body = _strip_comment(raw)
        if not body.strip():
            continue
        indent = len(body) - len(body.lstrip(" "))
        lines.append((indent, body.strip()))
    return lines


def _is_list_item(text: str) -> bool:
    return text == "-" or text.startswith("- ")


def _parse_map(lines, i: int, indent: int):
    result = {}
    while i < len(lines):
        ind, txt = lines[i]
        if ind < indent:
            break
        if ind > indent:
            raise ValueError("缩进异常: 第 %d 行 '%s'" % (i + 1, txt))
        if _is_list_item(txt):
            break
        if ":" not in txt:
            raise ValueError("缺少冒号: '%s'" % txt)
        key, _, rest = txt.partition(":")
        key = key.strip().strip("\"'")
        rest = rest.strip()
        i += 1
        if rest:
            result[key] = _scalar(rest)
            continue
        if i < len(lines) and lines[i][0] > indent:
            result[key], i = _parse_block(lines, i, lines[i][0])
        elif i < len(lines) and lines[i][0] == indent and _is_list_item(lines[i][1]):
            result[key], i = _parse_list(lines, i, indent)
        else:
            result[key] = None
    return result, i


def _parse_list(lines, i: int, indent: int):
    result = []
    while i < len(lines):
        ind, txt = lines[i]
        if ind != indent or not _is_list_item(txt):
            break
        rest = txt[1:].strip()
        i += 1
        if not rest:
            if i < len(lines) and lines[i][0] > indent:
                val, i = _parse_block(lines, i, lines[i][0])
            else:
                val = None
            result.append(val)
        else:
            result.append(_scalar(rest))
    return result, i


def _parse_block(lines, i: int, indent: int):
    if i >= len(lines):
        return None, i
    if _is_list_item(lines[i][1]):
        return _parse_list(lines, i, indent)
    return _parse_map(lines, i, indent)


def loads(text: str) -> Any:
    lines = _prepare(text)
    if not lines:
        return {}
    value, _ = _parse_block(lines, 0, lines[0][0])
    return value


def load(path: str) -> Any:
    with open(path, "r", encoding="utf-8") as fh:
        return loads(fh.read())


def _fmt_scalar(value: Any) -> str:
    if value is None:
        return "null"
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        return repr(round(value, 6))
    if isinstance(value, (int, str)):
        return str(value)
    raise TypeError("不支持的标量类型: %r" % (type(value),))


def dumps(obj: Any, indent: int = 0) -> str:
    pad = " " * indent
    if isinstance(obj, dict):
        out = []
        for key, val in obj.items():
            if isinstance(val, (dict, list)) and val:
                out.append("%s%s:" % (pad, key))
                out.append(dumps(val, indent + 2))
            elif isinstance(val, (dict, list)):
                out.append("%s%s: %s" % (pad, key, "{}" if isinstance(val, dict) else "[]"))
            else:
                out.append("%s%s: %s" % (pad, key, _fmt_scalar(val)))
        return "\n".join(out)
    if isinstance(obj, list):
        out = []
        for item in obj:
            if isinstance(item, (dict, list)) and item:
                out.append("%s-" % pad)
                out.append(dumps(item, indent + 2))
            else:
                out.append("%s- %s" % (pad, _fmt_scalar(item)))
        return "\n".join(out)
    return "%s%s" % (pad, _fmt_scalar(obj))


def dump(obj: Any, path: str) -> None:
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(dumps(obj))
        fh.write("\n")

