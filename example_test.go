package kinematics_test

import (
	"fmt"
	"github.com/koeng101/armos/utils/kinematics"
)

func ExampleForwardKinematics() {
	angles := kinematics.StepperTheta{10, 1, 1, 0, 0, 0}
	coordinates := kinematics.ForwardKinematics(angles, kinematics.AR3DhParameters)

	fmt.Println(coordinates)
	// Output: {-101.74590611879692 -65.96805988175777 -322.27756822304093 0.06040824945687102 -0.20421099379003957 0.2771553334491873 0.9369277637862541}
}

func ExampleInverseKinematics() {
	coordinates := kinematics.XyzWxyz{-100, 250, 250, 0.4007833787652043, -0.021233218878182854, 0.9086418268616911, 0.41903052745255764}
	angles, _ := kinematics.InverseKinematics(coordinates, kinematics.AR3DhParameters)

	fmt.Println(angles)
	// Output: {1.8462740950010432 0.3416721655970939 -2.313720459511564 -1.7765008677785283 2.2218097507147707 1.2318789996199948}
}
