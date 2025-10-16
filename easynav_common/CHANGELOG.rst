^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package easynav_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.3 (2025-10-12)
------------------
* New Constructor for PointPerceptionsView with only one perception
* Fix tf2_ros deprecation warnings
* Fix multiple perceptions
* Update defaul group var name
* Fix support for different perceptions types in blackboard
* Verbose fail in set. Set wit std::shared_ptr
* Contributors: Francisco Miguel Moreno Olivo, Francisco Martín Rico

0.1.2 (2025-09-26)
------------------
* Remove some yaets traces
* Add operation to include new points in the PointPerceptionView pipeline
* Trigger planner if new goals appear
* Minor improvements (refactor and RT option)
* [WIP] Blackboard for NavSate
* Simplify blackboard with two operations
* Remove atomics
* Remove get_ref
* Refactor Perceptions
* BlacKboard v3
* Redesign with NavState
* Blackboard for NavState
* Improving concurrency with atomic actions
* Yaets traces
* Blackboard working
* Remove pcl_conversions in target_link_libraries
* Improving concurrency with atomic actions
* Fix ament_target_dependencies_deprecation
* Install bins in correct place
* Fix ament_target_dependencies_deprecation
* Remove timeout
* Fix bug when disabling tracing. Rename tracing CMake option
* Updated Doxygen description headers
* easynav_interfaces and initial approach for a goal manager
* NavigationControl interface and GoalManager
* [WIP] System Execution
* RTTFBuffer. Dictionary maps.
* Improve PerceptionsOps to PerceptionsOpsView
* Add fused_perceptions method
* Add chained operations in perceptions
* Changes to load plugins
* Isolate Perceptions type from sensor utility functions
* Base interfaces for module algorithms
* Works on Map Manager Types
* Install runtime binary targets under lib
* Add NavState structure
* Move common base types to easynav_common
* Rename namespaces to easynav
* Refactoring to nodes
* Contributors: Francisco Martín Rico, Francisco Miguel Moreno Olivo, Juan Carlos Manzanares Serrano
