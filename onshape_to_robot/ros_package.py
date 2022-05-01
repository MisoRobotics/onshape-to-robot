"""Generate a launch file for ROS 2."""
import os
from collections import defaultdict
from enum import Enum
from pathlib import Path
from typing import List
from warnings import warn

import yaml
from jinja2 import Environment, FileSystemLoader, PackageLoader, select_autoescape


def _validate_package_name(package_name: str) -> None:
    if not package_name.endswith("description"):
        warn(f"Package name {package_name} does not end with '_description'.")


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
    subpath: Path,
    package_name: str,
    model_name: str,
    output_dir: Path = Path.cwd(),
) -> Path:
    """Render the specified template and write to an output file.

    Args:
        filename: Name of the template file from the templates folder.
        subpath: Subpath at which to place output file.
        package_name: Name of the generated catkin/ament package.
        model_name: The name of the model to generate.
        output_dir: Directory into which to place generated files.

    Returns:
        Path to the generated file.

    """
    if not output_dir.is_dir():
        raise ValueError(f"Specified output_dir is not a directory: {output_dir}")
    output_filepath = output_dir / subpath
    output_dir = Path(os.path.dirname(output_filepath))
    output_dir.mkdir(parents=True, exist_ok=True)
    rendered_content = render_template(filename, package_name, model_name)
    with open(output_filepath, "w", encoding="utf-8") as stream:
        stream.write(rendered_content)
    return output_filepath


def get_templates(
    package_type: str, model_type: str, **template_parameters
) -> List[dict]:
    """Return templates for specified package type.

    Combine files specific to both package type and model format.

    Args:
        package_type: The type of package to generate like "ament".
        model_format: The model format, i.e., 'urdf' or 'sdf'.
        **template_parameters: Parameters for rendering template config.

    """
    loader = FileSystemLoader(searchpath=os.path.dirname(__file__))
    env = Environment(loader=loader)
    template = env.get_template("ros_package.yaml")
    data = yaml.load(template.render(**template_parameters), Loader=yaml.Loader)
    package_templates = defaultdict(list)
    package_templates.update(
        {item["name"]: item["files"] for item in data["package_types"]}
    )
    model_templates = defaultdict(list)
    model_templates.update(
        {item["name"]: item["files"] for item in data["model_formats"]}
    )
    return package_templates[package_type] + model_templates[model_type]


def generate_package(
    package_name: str,
    package_type: str,
    model_name: str,
    model_format: str,
    output_dir: Path = Path.cwd(),
) -> List[Path]:
    """Generate a catkin/ament package for ROS.

    Args:
        package_name: Name of the generated package.
        package_type: The type of package to generate like "ament".
        model_name: The name of the model to generate.
        model_format: The model format, i.e., 'urdf' or 'sdf'.
        output_dir: Directory into which to place generated files.

    """
    templates = get_templates(package_type, model_format, model_name=model_name)
    generated_files = [
        process_template(
            item["source"], item["destination"], package_name, model_name, output_dir
        )
        for item in templates
    ]
    return generated_files
