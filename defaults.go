package kinematics

import (
	"math"
)

// Denavit-Hartenberg Parameters of AR3 provided by AR2 Version 2.0 software
// executable files from https://www.anninrobotics.com/downloads
// Those parameters are the same between the AR2 and AR3.
// J1 https://www.omc-stepperonline.com/download/17HS15-1684D-HG10-AR3.pdf
// J2 https://www.omc-stepperonline.com/download/23HS22-2804D-HG50-AR3.pdf
// J3 https://www.omc-stepperonline.com/download/17HS15-1684D-HG50-AR3.pdf
// J4 https://www.omc-stepperonline.com/download/11HS20-0674D-PG14-AR3.pdf
// J5 https://www.omc-stepperonline.com/download/17LS19-1684E-200G-AR3.pdf
// J6 https://www.omc-stepperonline.com/download/14HS11-1004D-PG19-AR3.pdf
var AR3DhParameters DhParameters = DhParameters{
	ThetaOffsets:   []float64{0, 0, -math.Pi / 2, 0, 0, math.Pi},
	AlphaValues:    []float64{-(math.Pi / 2), 0, math.Pi / 2, -(math.Pi / 2), math.Pi / 2, 0},
	AValues:        []float64{64.2, 305, 0, 0, 0, 0},
	DValues:        []float64{169.77, 0, 0, -222.63, 0, -36.25},
	StepperLimits:  []float64{15200, 7300, 7850, 15200, 4575, 6625},
	StepsPerRadian: []float64{(1.8 / 10) * (math.Pi / 180), (1.8 / 50) * (math.Pi / 180), (1.8 / 50) * (math.Pi / 180), (1.8 / (13 + (212 / 289))) * (math.Pi / 180), (1.8) * (math.Pi / 180), (1.8 / (19 + (38 / 137))) * (math.Pi / 180)},
}
