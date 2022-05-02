"""Generate a launch file for ROS 2."""
import os
from pathlib import Path
from typing import List
from warnings import warn

from jinja2 import Environment, PackageLoader, select_autoescape


def _validate_package_name(package_name: str) -> None:
    if not package_name.endswith("description"):
        warn("Package name {} does not end with '_description'.".format(package_name))


def render_template(filename: str, package_name: str, model_name: str) -> str:
    """Render the specified template file and return as a string.

    Args:
        filename: Name of the template file from the templates folder.
        package_name: Name of the generated catkin/ament package.
        model_name: The name of the model to generate.

    Returns:
        The contents generated from rendering the passed template file.

    """
    _validate_package_name(package_name)
    if not filename.endswith(".jinja"):
        filename += ".jinja"
    env = Environment(
        loader=PackageLoader(os.path.basename(os.path.dirname(__file__))),
        autoescape=select_autoescape(),
        keep_trailing_newline=True,
    )
    template = env.get_template(filename)
    rendered_content = template.render(
        package_name=package_name,
        model_name=model_name,
    )
    return rendered_content


def process_template(
    filename: str,
    subdir: Path,
    package_name: str,
    model_name: str,
    output_dir: Path = Path.cwd(),
) -> Path:
    """Render the specified template and write to an output file.

    Args:
        filename: Name of the template file from the templates folder.
        subdir: Subdirectory into which to place output file.
        package_name: Name of the generated catkin/ament package.
        model_name: The name of the model to generate.
        output_dir: Directory into which to place generated files.

    Returns:
        Path to the generated file.

    """
    if not output_dir.is_dir():
        raise ValueError(
            "Specified output_dir is not a directory: {}".format(output_dir)
        )
    if not filename.endswith(".jinja"):
        filename += ".jinja"
    output_dir /= subdir
    output_dir.mkdir(mode=0o755, exist_ok=True)
    output_filepath = output_dir / filename[: -len(".jinja")]
    rendered_content = render_template(filename, package_name, model_name)
    with open(output_filepath, "w", encoding="utf-8") as stream:
        stream.write(rendered_content)
    return output_filepath


def generate_ament_package(
    package_name: str,
    model_name: str,
    model_format: str,
    output_dir: Path = Path.cwd(),
) -> List[Path]:
    """Generate an ament package for ROS 2.

    Args:
        package_name: Name of the generated catkin/ament package.
        model_name: The name of the model to generate.
        model_format: The model format, i.e., 'urdf' or 'sdf'.
        output_dir: Directory into which to place generated files.

    """
    templates = {
        "CMakeLists.txt": Path("."),
        "package.xml": Path("."),
        "view_robot.launch.py": Path("launch"),
        "view_robot.rviz": Path("rviz"),
    }
    if model_format == "sdf":
        templates["model.config"] = Path("models") / model_name
    generated_files = [
        process_template(filename, subdir, package_name, model_name, output_dir)
        for filename, subdir in templates.items()
    ]
    return generated_files
