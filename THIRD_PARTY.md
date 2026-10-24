# Third-party material and provenance

The project's own code and documentation are licensed under the [MIT License](LICENSE). That license does not replace the separate notices and terms for the bundled scenarios, models, or external dependencies listed below.

| Material | Provenance / notice |
|---|---|
| `scenarios/intas/` (except `models/`) | Adapted from [InTAS](https://github.com/silaslobo/InTAS), by Silas Lobo, Stefan Neumeier, Evelio Fernández, and Christian Facchi. Existing GPL version 3 or later notices are retained; the complete upstream license is included in [COPYING](scenarios/intas/COPYING). See the local [provenance notice](scenarios/intas/README.md) for project adaptations. |
| `scenarios/xgrid/` | Project-authored synthetic SUMO grid scenario, partition data, routes, models, and reference output, distributed under the [MIT License](scenarios/xgrid/LICENSE). Model-training scripts are not included. |
| `scenarios/*/models/` | Project-authored pretrained XGBoost models, distributed under MIT: [xgrid](scenarios/xgrid/LICENSE), [InTAS models](scenarios/intas/models/LICENSE). Their embedded save-version metadata is documented in the README. |
| SUMO | External dependency, built from the pinned `tem-kurbanov/sumo` fork. Its license files remain in that source tree; SUMO is not vendored here. |
| XGBoost | External dependency, built from the checksum-verified 3.0.5 source release. Its Apache 2.0 license and bundled component notices remain in the extracted source tree; XGBoost is not vendored here. |

The active configurations select `xgrid/5_typed.rou.xml` and `intas/test_typed.rou.xml`. The unused `intas/combined_routes.rou.xml` and its historical `scenario_original.sumocfg` are excluded from Git and retained locally. Other supplied route files and reference outputs remain available. The InTAS paper is available at [doi:10.52825/scp.v1i.102](https://doi.org/10.52825/scp.v1i.102).
