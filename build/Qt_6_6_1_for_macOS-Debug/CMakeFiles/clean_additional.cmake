# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "CMakeFiles/jeepApp_autogen.dir/AutogenUsed.txt"
  "CMakeFiles/jeepApp_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/application/CMakeFiles/QuickStudioApplication_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/application/CMakeFiles/QuickStudioApplication_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/application/CMakeFiles/QuickStudioApplication_resources_1_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/application/CMakeFiles/QuickStudioApplication_resources_1_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/application/CMakeFiles/QuickStudioApplicationplugin_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/application/CMakeFiles/QuickStudioApplicationplugin_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/application/CMakeFiles/QuickStudioApplicationplugin_init_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/application/CMakeFiles/QuickStudioApplicationplugin_init_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/application/QuickStudioApplication_autogen"
  "_deps/ds-build/src/imports/application/QuickStudioApplication_resources_1_autogen"
  "_deps/ds-build/src/imports/application/QuickStudioApplicationplugin_autogen"
  "_deps/ds-build/src/imports/application/QuickStudioApplicationplugin_init_autogen"
  "_deps/ds-build/src/imports/components/CMakeFiles/QuickStudioComponents_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/components/CMakeFiles/QuickStudioComponents_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/components/CMakeFiles/QuickStudioComponents_qmlcache_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/components/CMakeFiles/QuickStudioComponents_qmlcache_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/components/CMakeFiles/QuickStudioComponents_resources_1_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/components/CMakeFiles/QuickStudioComponents_resources_1_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/components/CMakeFiles/QuickStudioComponents_resources_2_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/components/CMakeFiles/QuickStudioComponents_resources_2_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/components/CMakeFiles/QuickStudioComponentsplugin_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/components/CMakeFiles/QuickStudioComponentsplugin_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/components/CMakeFiles/QuickStudioComponentsplugin_init_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/components/CMakeFiles/QuickStudioComponentsplugin_init_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/components/QuickStudioComponents_autogen"
  "_deps/ds-build/src/imports/components/QuickStudioComponents_qmlcache_autogen"
  "_deps/ds-build/src/imports/components/QuickStudioComponents_resources_1_autogen"
  "_deps/ds-build/src/imports/components/QuickStudioComponents_resources_2_autogen"
  "_deps/ds-build/src/imports/components/QuickStudioComponentsplugin_autogen"
  "_deps/ds-build/src/imports/components/QuickStudioComponentsplugin_init_autogen"
  "_deps/ds-build/src/imports/effects_qt6/CMakeFiles/QuickStudioEffects_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/effects_qt6/CMakeFiles/QuickStudioEffects_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/effects_qt6/CMakeFiles/QuickStudioEffects_resources_1_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/effects_qt6/CMakeFiles/QuickStudioEffects_resources_1_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/effects_qt6/CMakeFiles/QuickStudioEffects_resources_2_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/effects_qt6/CMakeFiles/QuickStudioEffects_resources_2_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/effects_qt6/CMakeFiles/QuickStudioEffectsplugin_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/effects_qt6/CMakeFiles/QuickStudioEffectsplugin_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/effects_qt6/CMakeFiles/QuickStudioEffectsplugin_init_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/effects_qt6/CMakeFiles/QuickStudioEffectsplugin_init_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/effects_qt6/QuickStudioEffects_autogen"
  "_deps/ds-build/src/imports/effects_qt6/QuickStudioEffects_resources_1_autogen"
  "_deps/ds-build/src/imports/effects_qt6/QuickStudioEffects_resources_2_autogen"
  "_deps/ds-build/src/imports/effects_qt6/QuickStudioEffectsplugin_autogen"
  "_deps/ds-build/src/imports/effects_qt6/QuickStudioEffectsplugin_init_autogen"
  "_deps/ds-build/src/imports/flowview/CMakeFiles/FlowView_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/flowview/CMakeFiles/FlowView_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/flowview/CMakeFiles/FlowView_resources_1_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/flowview/CMakeFiles/FlowView_resources_1_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/flowview/CMakeFiles/FlowViewplugin_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/flowview/CMakeFiles/FlowViewplugin_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/flowview/CMakeFiles/FlowViewplugin_init_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/flowview/CMakeFiles/FlowViewplugin_init_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/flowview/FlowView_autogen"
  "_deps/ds-build/src/imports/flowview/FlowView_resources_1_autogen"
  "_deps/ds-build/src/imports/flowview/FlowViewplugin_autogen"
  "_deps/ds-build/src/imports/flowview/FlowViewplugin_init_autogen"
  "_deps/ds-build/src/imports/logichelper/CMakeFiles/QuickStudioLogicHelper_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/logichelper/CMakeFiles/QuickStudioLogicHelper_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/logichelper/CMakeFiles/QuickStudioLogicHelper_qmlcache_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/logichelper/CMakeFiles/QuickStudioLogicHelper_qmlcache_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/logichelper/CMakeFiles/QuickStudioLogicHelper_resources_1_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/logichelper/CMakeFiles/QuickStudioLogicHelper_resources_1_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/logichelper/CMakeFiles/QuickStudioLogicHelper_resources_2_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/logichelper/CMakeFiles/QuickStudioLogicHelper_resources_2_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/logichelper/CMakeFiles/QuickStudioLogicHelperplugin_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/logichelper/CMakeFiles/QuickStudioLogicHelperplugin_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/logichelper/CMakeFiles/QuickStudioLogicHelperplugin_init_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/logichelper/CMakeFiles/QuickStudioLogicHelperplugin_init_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/logichelper/QuickStudioLogicHelper_autogen"
  "_deps/ds-build/src/imports/logichelper/QuickStudioLogicHelper_qmlcache_autogen"
  "_deps/ds-build/src/imports/logichelper/QuickStudioLogicHelper_resources_1_autogen"
  "_deps/ds-build/src/imports/logichelper/QuickStudioLogicHelper_resources_2_autogen"
  "_deps/ds-build/src/imports/logichelper/QuickStudioLogicHelperplugin_autogen"
  "_deps/ds-build/src/imports/logichelper/QuickStudioLogicHelperplugin_init_autogen"
  "_deps/ds-build/src/imports/multitext/CMakeFiles/QuickStudioMultiText_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/multitext/CMakeFiles/QuickStudioMultiText_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/multitext/CMakeFiles/QuickStudioMultiText_resources_1_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/multitext/CMakeFiles/QuickStudioMultiText_resources_1_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/multitext/CMakeFiles/QuickStudioMultiText_resources_2_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/multitext/CMakeFiles/QuickStudioMultiText_resources_2_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/multitext/CMakeFiles/QuickStudioMultiTextplugin_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/multitext/CMakeFiles/QuickStudioMultiTextplugin_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/multitext/CMakeFiles/QuickStudioMultiTextplugin_init_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/multitext/CMakeFiles/QuickStudioMultiTextplugin_init_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/multitext/QuickStudioMultiText_autogen"
  "_deps/ds-build/src/imports/multitext/QuickStudioMultiText_resources_1_autogen"
  "_deps/ds-build/src/imports/multitext/QuickStudioMultiText_resources_2_autogen"
  "_deps/ds-build/src/imports/multitext/QuickStudioMultiTextplugin_autogen"
  "_deps/ds-build/src/imports/multitext/QuickStudioMultiTextplugin_init_autogen"
  "_deps/ds-build/src/imports/tools/eventsimulator/CMakeFiles/QuickStudioEventSimulator_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/tools/eventsimulator/CMakeFiles/QuickStudioEventSimulator_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/tools/eventsimulator/CMakeFiles/QuickStudioEventSimulator_resources_1_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/tools/eventsimulator/CMakeFiles/QuickStudioEventSimulator_resources_1_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/tools/eventsimulator/CMakeFiles/QuickStudioEventSimulator_resources_2_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/tools/eventsimulator/CMakeFiles/QuickStudioEventSimulator_resources_2_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/tools/eventsimulator/CMakeFiles/QuickStudioEventSimulatorplugin_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/tools/eventsimulator/CMakeFiles/QuickStudioEventSimulatorplugin_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/tools/eventsimulator/CMakeFiles/QuickStudioEventSimulatorplugin_init_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/tools/eventsimulator/CMakeFiles/QuickStudioEventSimulatorplugin_init_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/tools/eventsimulator/QuickStudioEventSimulator_autogen"
  "_deps/ds-build/src/imports/tools/eventsimulator/QuickStudioEventSimulator_resources_1_autogen"
  "_deps/ds-build/src/imports/tools/eventsimulator/QuickStudioEventSimulator_resources_2_autogen"
  "_deps/ds-build/src/imports/tools/eventsimulator/QuickStudioEventSimulatorplugin_autogen"
  "_deps/ds-build/src/imports/tools/eventsimulator/QuickStudioEventSimulatorplugin_init_autogen"
  "_deps/ds-build/src/imports/tools/eventsystem/CMakeFiles/QuickStudioEventSystem_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/tools/eventsystem/CMakeFiles/QuickStudioEventSystem_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/tools/eventsystem/CMakeFiles/QuickStudioEventSystem_resources_1_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/tools/eventsystem/CMakeFiles/QuickStudioEventSystem_resources_1_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/tools/eventsystem/CMakeFiles/QuickStudioEventSystem_resources_2_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/tools/eventsystem/CMakeFiles/QuickStudioEventSystem_resources_2_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/tools/eventsystem/CMakeFiles/QuickStudioEventSystemplugin_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/tools/eventsystem/CMakeFiles/QuickStudioEventSystemplugin_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/tools/eventsystem/CMakeFiles/QuickStudioEventSystemplugin_init_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/tools/eventsystem/CMakeFiles/QuickStudioEventSystemplugin_init_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/tools/eventsystem/QuickStudioEventSystem_autogen"
  "_deps/ds-build/src/imports/tools/eventsystem/QuickStudioEventSystem_resources_1_autogen"
  "_deps/ds-build/src/imports/tools/eventsystem/QuickStudioEventSystem_resources_2_autogen"
  "_deps/ds-build/src/imports/tools/eventsystem/QuickStudioEventSystemplugin_autogen"
  "_deps/ds-build/src/imports/tools/eventsystem/QuickStudioEventSystemplugin_init_autogen"
  "_deps/ds-build/src/imports/utils/CMakeFiles/QuickStudioUtils_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/utils/CMakeFiles/QuickStudioUtils_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/utils/CMakeFiles/QuickStudioUtils_qmlcache_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/utils/CMakeFiles/QuickStudioUtils_qmlcache_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/utils/CMakeFiles/QuickStudioUtils_resources_1_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/utils/CMakeFiles/QuickStudioUtils_resources_1_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/utils/CMakeFiles/QuickStudioUtils_resources_2_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/utils/CMakeFiles/QuickStudioUtils_resources_2_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/utils/CMakeFiles/QuickStudioUtilsplugin_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/utils/CMakeFiles/QuickStudioUtilsplugin_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/utils/CMakeFiles/QuickStudioUtilsplugin_init_autogen.dir/AutogenUsed.txt"
  "_deps/ds-build/src/imports/utils/CMakeFiles/QuickStudioUtilsplugin_init_autogen.dir/ParseCache.txt"
  "_deps/ds-build/src/imports/utils/QuickStudioUtils_autogen"
  "_deps/ds-build/src/imports/utils/QuickStudioUtils_qmlcache_autogen"
  "_deps/ds-build/src/imports/utils/QuickStudioUtils_resources_1_autogen"
  "_deps/ds-build/src/imports/utils/QuickStudioUtils_resources_2_autogen"
  "_deps/ds-build/src/imports/utils/QuickStudioUtilsplugin_autogen"
  "_deps/ds-build/src/imports/utils/QuickStudioUtilsplugin_init_autogen"
  "content/CMakeFiles/content_autogen.dir/AutogenUsed.txt"
  "content/CMakeFiles/content_autogen.dir/ParseCache.txt"
  "content/CMakeFiles/content_qmlcache_autogen.dir/AutogenUsed.txt"
  "content/CMakeFiles/content_qmlcache_autogen.dir/ParseCache.txt"
  "content/CMakeFiles/content_resources_1_autogen.dir/AutogenUsed.txt"
  "content/CMakeFiles/content_resources_1_autogen.dir/ParseCache.txt"
  "content/CMakeFiles/content_resources_2_autogen.dir/AutogenUsed.txt"
  "content/CMakeFiles/content_resources_2_autogen.dir/ParseCache.txt"
  "content/CMakeFiles/contentplugin_autogen.dir/AutogenUsed.txt"
  "content/CMakeFiles/contentplugin_autogen.dir/ParseCache.txt"
  "content/CMakeFiles/contentplugin_init_autogen.dir/AutogenUsed.txt"
  "content/CMakeFiles/contentplugin_init_autogen.dir/ParseCache.txt"
  "content/content_autogen"
  "content/content_qmlcache_autogen"
  "content/content_resources_1_autogen"
  "content/content_resources_2_autogen"
  "content/contentplugin_autogen"
  "content/contentplugin_init_autogen"
  "imports/jeep/CMakeFiles/jeep_autogen.dir/AutogenUsed.txt"
  "imports/jeep/CMakeFiles/jeep_autogen.dir/ParseCache.txt"
  "imports/jeep/CMakeFiles/jeep_qmlcache_autogen.dir/AutogenUsed.txt"
  "imports/jeep/CMakeFiles/jeep_qmlcache_autogen.dir/ParseCache.txt"
  "imports/jeep/CMakeFiles/jeep_resources_1_autogen.dir/AutogenUsed.txt"
  "imports/jeep/CMakeFiles/jeep_resources_1_autogen.dir/ParseCache.txt"
  "imports/jeep/CMakeFiles/jeep_resources_2_autogen.dir/AutogenUsed.txt"
  "imports/jeep/CMakeFiles/jeep_resources_2_autogen.dir/ParseCache.txt"
  "imports/jeep/CMakeFiles/jeepplugin_autogen.dir/AutogenUsed.txt"
  "imports/jeep/CMakeFiles/jeepplugin_autogen.dir/ParseCache.txt"
  "imports/jeep/CMakeFiles/jeepplugin_init_autogen.dir/AutogenUsed.txt"
  "imports/jeep/CMakeFiles/jeepplugin_init_autogen.dir/ParseCache.txt"
  "imports/jeep/jeep_autogen"
  "imports/jeep/jeep_qmlcache_autogen"
  "imports/jeep/jeep_resources_1_autogen"
  "imports/jeep/jeep_resources_2_autogen"
  "imports/jeep/jeepplugin_autogen"
  "imports/jeep/jeepplugin_init_autogen"
  "jeepApp_autogen"
  )
endif()
