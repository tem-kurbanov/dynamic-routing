# InTAS scenario provenance

This directory contains a project adaptation of [InTAS — The Ingolstadt Traffic Scenario for SUMO](https://github.com/silaslobo/InTAS), developed by Silas Lobo, Stefan Neumeier, Evelio Fernández, and Christian Facchi.

The scenario-derived network, configurations, routes, polygon data, and graph partitions retain the upstream GPL terms. Existing notices permit GNU GPL version 3 or later. The complete upstream license is included in [COPYING](COPYING), copied from upstream license blob `f288702d2fa16d3cdf0035b15a9fcbc552cd88e7`.

Project adaptations supplied for the traffic-routing research include the reset road network, selected and typed vehicle routes, simulation configuration, and precomputed partitions for one to four area agents. These are modified scenario inputs, not an unmodified upstream release. The supplied modified files date from 2025; exact upstream-base and transformation histories are not available. This provenance and licensing notice was added on 2026-10-23.

The active configuration is `scenario.sumocfg`, which uses `reset.net.xml`, `test_typed.rou.xml`, and `InTAS.poly.xml`. An unused combined-route file and its historical configuration are retained locally but excluded from Git.

The project-authored trained model files under `models/` have their own [MIT license](models/LICENSE). They are separate prediction artifacts; they do not replace the license of the InTAS scenario inputs.

Reference: Silas Lobo, Stefan Neumeier, Evelio M. G. Fernandez, Christian Facchi, *InTAS — The Ingolstadt Traffic Scenario for SUMO*, SUMO Conference Proceedings, volume 1, pages 73–92. [DOI: 10.52825/scp.v1i.102](https://doi.org/10.52825/scp.v1i.102).
