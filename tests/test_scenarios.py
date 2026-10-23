"""Check bundled scenario inputs without running SUMO or loading native models."""
import csv
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


class ScenarioTests(unittest.TestCase):
    def test_configuration_inputs_exist(self):
        for name in ("xgrid", "intas"):
            scenario = ROOT / "scenarios" / name
            config = ET.parse(scenario / "scenario.sumocfg").getroot()
            for tag in ("net-file", "route-files", "additional-files"):
                node = config.find("input/" + tag)
                if node is not None:
                    for value in node.attrib["value"].split(","):
                        with self.subTest(scenario=name, input=value):
                            self.assertTrue((scenario / value).is_file())

    def test_partition_indices_and_adjacency(self):
        for name in ("xgrid", "intas"):
            for agents in range(1, 5):
                with self.subTest(scenario=name, agents=agents):
                    graph = ROOT / "scenarios" / name / "graphs"
                    with (graph / f"nodes{agents}.csv").open() as stream:
                        nodes = list(csv.reader(stream, delimiter=";"))[1:]
                    with (graph / f"edges{agents}.csv").open() as stream:
                        edges = list(csv.reader(stream, delimiter=";"))[1:]
                    self.assertEqual([int(row[0]) for row in nodes], list(range(len(nodes))))
                    self.assertEqual([int(row[0]) for row in edges], list(range(len(edges))))
                    for row in edges:
                        self.assertTrue(0 <= int(row[1]) < len(nodes))
                        self.assertTrue(0 <= int(row[2]) < len(nodes))
                        self.assertTrue(0 < int(row[-1]) < (1 << agents))
                        for field in row[-3:-1]:
                            for edge in json.loads(field):
                                self.assertTrue(0 <= edge < len(edges))

    def test_log_models_match_predictor_features(self):
        for name in ("xgrid", "intas"):
            for model, features in (("density", 8), ("speed", 6), ("fuel", 5)):
                with self.subTest(scenario=name, model=model):
                    path = ROOT / "scenarios" / name / "models/log" / f"{model}_60.json"
                    data = json.loads(path.read_text())
                    self.assertEqual(int(data["learner"]["learner_model_param"]["num_feature"]), features)
                    self.assertEqual(data["version"], [3, 0, 0] if name == "xgrid" else [3, 0, 5])


if __name__ == "__main__":
    unittest.main()
