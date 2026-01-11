cd E:\Repository\GT-karny\GT-SimulatorIntegration
mkdir -Force generated\osi3 | Out-Null

$protoc = "E:\Repository\vcpkg\installed\x64-windows\tools\protobuf\protoc.exe"
$osi    = "E:\Repository\GT-karny\GT-SimulatorIntegration\thirdparty\open-simulation-interface-3.5.0"
$out    = "E:\Repository\GT-karny\GT-SimulatorIntegration\generated\osi3"

& $protoc `
  -I $osi `
  --cpp_out=$out `
  $osi\osi_common.proto `
  $osi\osi_datarecording.proto `
  $osi\osi_detectedtrafficsign.proto `
  $osi\osi_detectedtrafficlight.proto `
  $osi\osi_detectedroadmarking.proto `
  $osi\osi_detectedlane.proto `
  $osi\osi_detectedobject.proto `
  $osi\osi_detectedoccupant.proto `
  $osi\osi_environment.proto `
  $osi\osi_groundtruth.proto `
  $osi\osi_hostvehicledata.proto `
  $osi\osi_trafficsign.proto `
  $osi\osi_trafficlight.proto `
  $osi\osi_trafficupdate.proto `
  $osi\osi_trafficcommand.proto `
  $osi\osi_referenceline.proto `
  $osi\osi_roadmarking.proto `
  $osi\osi_lane.proto `
  $osi\osi_logicallane.proto `
  $osi\osi_featuredata.proto `
  $osi\osi_logicaldetectiondata.proto `
  $osi\osi_object.proto `
  $osi\osi_occupant.proto `
  $osi\osi_sensordata.proto `
  $osi\osi_sensorviewconfiguration.proto `
  $osi\osi_sensorspecific.proto `
  $osi\osi_sensorview.proto
