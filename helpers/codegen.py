"""General Tools for code generation."""

import os


def write_init_file(module_dir: str, class_names: list[str]):
    """Generate __init__.py to imports all classes and sets __all__, alphabetically."""
    class_names = sorted(class_names)  # Alphabetical order

    lines = [
        '"""Init file for enums package."""',
        "",
    ]
    for name in class_names:
        lines.append(f"from .{name.lower()} import {name}")
    lines.append("")
    lines.append("__all__ = [")
    for name in class_names:
        lines.append(f'    "{name}",')
    lines.append("]")

    init_path = os.path.join(module_dir, "__init__.py")
    with open(init_path, "w") as f:
        f.write("\n".join(lines))
    print(f"✅ __init__.py written to {init_path}")
