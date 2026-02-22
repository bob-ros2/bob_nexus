import argparse
import os
import re

from dotenv import dotenv_values


class TemplateEngine:
    def __init__(self, env_path=None, extra_vars=None):
        self.env_vars = {}
        if env_path and os.path.exists(env_path):
            self.env_vars = dotenv_values(env_path)

        # Merge with system environment (system env takes precedence)
        self.env_vars.update(dict(os.environ))

        if extra_vars:
            self.env_vars.update(extra_vars)

    def resolve_placeholder(self, match):
        """
        Resolves ${VAR} or ${VAR:-default}
        """
        full_var = match.group(1)
        if ":-" in full_var:
            var_name, default_value = full_var.split(":-", 1)
        else:
            var_name = full_var
            default_value = ""

        value = self.env_vars.get(var_name)
        if value is not None and value != "":
            return value

        if ":-" in full_var:
            return default_value

        return f"${{{var_name}}}"

    def process_content(self, content):
        """
        Finds all ${...} and replaces them. Loops to handle nested resolutions.
        """
        # This matches ${VAR} or ${VAR:-DEFAULT} where the content does not contain braces.
        # By looping, we resolve inside-out.
        pattern = r"\$\{([^{}]+)\}"

        previous_content = None
        while content != previous_content:
            previous_content = content
            content = re.sub(pattern, self.resolve_placeholder, content)
        return content

    def process_file(self, input_path, output_path):
        """
        Reads input_path, processes templates, writes to output_path.
        """
        if not os.path.exists(input_path):
            raise FileNotFoundError(f"Template not found: {input_path}")

        with open(input_path, "r") as f:
            content = f.read()

        processed = self.process_content(content)

        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        with open(output_path, "w") as f:
            f.write(processed)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Mastermind Template Engine")
    parser.add_argument("input", help="Path to template file")
    parser.add_argument("output", help="Path to output file")
    parser.add_argument("--env", help="Path to .env file", default=".env")

    args = parser.parse_args()

    engine = TemplateEngine(args.env)
    try:
        engine.process_file(args.input, args.output)
        print(f"Successfully processed {args.input} -> {args.output}")
    except Exception as e:
        print(f"Error processing template: {e}")
        exit(1)
