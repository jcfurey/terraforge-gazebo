
from jinja2 import Environment, FileSystemLoader

from terraforge.utils.logging import logger


class SDFWorldBuilder:
    def __init__(self, template_dir):
        self.template_env = Environment(loader=FileSystemLoader(template_dir))
        logger.info(f"SDF World Builder initialized with template directory: {template_dir}")

    def render_world_template(self, *, heightmap_path=None, texture_path=None,
                              flat_normal_path=None,
                              buildings=None, trees=None, roads=None,
                              extent_meters=1000.0, height_amplitude=200.0,
                              terrain_z_offset=0.0):
        template = self.template_env.get_template('world_template.sdf.j2')
        rendered_sdf = template.render(
            heightmap_path=heightmap_path,
            texture_path=texture_path,
            flat_normal_path=flat_normal_path,
            buildings=buildings or [],
            trees=trees or [],
            roads=roads or [],
            extent_meters=extent_meters,
            height_amplitude=height_amplitude,
            terrain_z_offset=terrain_z_offset,
        )
        logger.info("SDF world template rendered.")
        return rendered_sdf

    def save_sdf_world_file(self, sdf_content, output_path):
        with open(output_path, 'w') as sdf_file:
            sdf_file.write(sdf_content)
        logger.info(f"SDF world file saved to {output_path}")
