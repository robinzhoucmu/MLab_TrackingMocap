#include "obj_reg.h"

ObjectReg::ObjectReg() {
}

void ObjectReg::Serialize(std::ostream & fout) {
}

void ObjectReg::Deserialize(std::istream& fin) {
}

void ObjectReg::ReadCaliMarkersFromMocap(MocapComm& mocap_comm) {
  cali_markers_pos.clear();
  Mocap::mocap_frame mocap_msg;
  mocap_comm.GetMocapFrame(&mocap_msg);
  
  // Extract cali markers(which NatNet will treat as unidenfied markers).
  int num_markers = mocap_msg.uid_markers.markers.size();
  // Check for size.
  assert(num_markers == 3);
  for (int i = 0; i < num_markers; ++i) {
    const geometry_msgs::Point& pt_mocap = mocap_msg.uid_markers.markers[i];
    Vec pt(3);
    pt[0] = pt_mocap.x;
    pt[1] = pt_mocap.y;
    pt[2] = pt_mocap.z;
    cali_markers_pos.push_back(pt);
  }
  FormCaliMarkerCoordinateFrame();
}

void ObjectReg::FormCaliMarkerCoordinateFrame() {
  int num_markers = cali_markers_pos.size();
  assert(num_markers == 3);
  int id_origin = -1;
  const double eps_dot_product = 0.1; 
  Vec axis_x;
  Vec axis_y;
  for (int i = 0; i < num_markers; ++i) {
    // Determine the ith point can be set as the origin.
    // Check the angle between axis is close to right angle or not.
    axis_x = cali_markers_pos[(i+1) % num_markers] - cali_markers_pos[i];
    axis_y = cali_markers_pos[(i+2) % num_markers] - cali_markers_pos[i];
    const double norm_axis_x = sqrt(axis_x * axis_x);
    const double norm_axis_y = sqrt(axis_y * axis_y);
    
    // Use the longer axis as the y axis.
    if (norm_axis_x > norm_axis_y) {
      Vec tmp = axis_x;
      axis_x = axis_y;
      axis_y = tmp;
    }

    // Check dot product.
    double dot_product = 
      (axis_x * axis_y) / (sqrt(axis_x * axis_x) * sqrt(axis_y * axis_y));  
    if (dot_product < eps_dot_product) {
      id_origin = i;
      break;
    }
  }
  double z[3] = {0, 0, 1};
  Vec axis_z(z,3);
  // axis_x and axis_y may not be perfectly perpendicular, we reset axis_x as 
  // the cross-product of axis_z and axis_y.
  // Todo(Jiaji): Find the nearest rotation matrix by projection.
  axis_x = axis_y ^ axis_z;
  // Form the RefFrame.
  RotMat rot_mat(axis_x, axis_y, axis_z);
  // Form the Translation.
  Vec trans(cali_markers_pos[id_origin]);
  // Update the homogenious transformation.
  tf_robot_calimarkers.setRotation(rot_mat);
  tf_robot_calimarkers.setTranslation(trans);
}


void ReadTractablePoseFromMocap(MocapComm& mocap_comm) {
}

void ComputeTransformation() {
}


int main(int argc, char* argv[]) {
  return 0;
}
