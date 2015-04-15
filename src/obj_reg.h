#ifndef OBJ_REJ_H
#define OBJ_REJ_H

// This header file defines the object registration class.
// ObjectReg instance contains two mocap readings to register the object.

#include <fstream>
#include <iostream>
#include <Mocap/mocap_comm.h>
#include <Mocap/mocap_frame.h>
#include <matVec/matVec.h>

class ObjectReg {
 public:
  ObjectReg();
  
  void Serialize(std::ostream& fout);
  void Deserialize(std::istream& fin);

  void ReadCaliMarkersFromMocap(const MocapComm& mocap_comm);
  void ReadTractablePoseFromMocap(const MocapComm& mocap_comm);
  
  void ComputeTransformation();

 private:
  std::vector<Vec> cali_markers_pos;
  // Transformation from robot base to motion capture tractable.
  HomogTransf tf_robot_mctractable;

  // Transformation from robot base to calibration markers.
  // If mocap publisher is calibrated to robot base, we could directly read off  
  // from mocap topic information. 
  // We will use this as one sample of known T_{R}^{obj}.
  HomogTransf tf_robot_calimarkers;

  // Transformation (T_{mct}^{obj}) from mocap frame tractable to local object frame. 
  // T_{R}^{mct} * T_{mct}^{obj} = T_{R}^{obj}.
  // T_{mct}^{obj} = inv(T_{R}^{mct}) * T_{R}^{obj}
  HomogTransf tf_mctractable_obj;
  
  // Pairwise distance matrix that characterize the rigid body for association when
  // mocap is tracking multiple objects.
  Mat cm_dis_mat;
  
};

#endif
