/*
* This file is automatically generated by px_generate_mixers.py - do not edit.
*/

#ifndef _MIXER_MULTI_TABLES
#define _MIXER_MULTI_TABLES

enum class MultirotorGeometry : MultirotorGeometryUnderlyingType {
	DODECA_BOTTOM_COX,             // Generic Dodecacopter in X coax configuration, bottom half (text key 6a)
	DODECA_TOP_COX,                // Generic Dodecacopter in X coax configuration, top half (text key 6m)
	HEX_COX,                       // Generic Hexacopter in coaxial configuration (text key 6c)
	HEX_PLUS,                      // Generic Hexacopter in + configuration (text key 6+)
	HEX_T,                         // Generic Hexacopter in T configuration (text key 6t)
	HEX_X,                         // Generic Hexacopter in X configuration (text key 6x)
	OCTA_COX,                      // GenericOctacopter in coax configuration (text key 8c)
	OCTA_COX_WIDE,                 // Generic Octacopter in wide coax configuration (text key 8cw)
	OCTA_PLUS,                     // Generic Octacopter in + configuration (text key 8+)
	OCTA_X,                        // Generic Octacopter in X configuration (text key 8x)
	QUAD_DEADCAT,                  // SK450 DeadCat Quadcopter, CG at intersection of rear arms (text key 4dc)
	QUAD_H,                        // Generic Quadcopter in H configuration (text key 4h)
	QUAD_PLUS,                     // Generic Quadcopter in + configuration (text key 4+)
	QUAD_S250AQ,                   // Spedix S250AQ Quadcopter (text key 4s)
	QUAD_VTAIL,                    // Quadcopter in Y configuration with rear props tilted at 45 degrees (text key 4vt)
	QUAD_WIDE,                     // Quadcopter in wide configuration. Same geometry as SK450 Deadcat except the CG is moved backward to load all motors equally (text key 4w)
	QUAD_X_CW,                     // Quadcopter in X configuration with clock-wise motor numbering (text key 4xcw)
	QUAD_X,                        // Generic Quadcopter in X configuration (text key 4x)
	QUAD_X_PUSHER,                 // Quadcopter in X configuration, with added pusher motor in the back (text key 4x1p)
	QUAD_Y,                        // Quadcopter in Y configuration with coax rear props (text key 4y)
	TRI_Y,                         // Tri Y (text key 3y)
	TWIN_ENGINE,                   // Twin engine (text key 2-)

	MAX_GEOMETRY
}; // enum class MultirotorGeometry

namespace {
static constexpr MultirotorMixer::Rotor _config_dodeca_bottom_cox[] {
	{ -0.333333,  0.000000,  3.333333,  0.000000,  0.000000, -0.166667 },
	{  0.333333, -0.000000, -3.333333,  0.000000,  0.000000, -0.166667 },
	{  0.166667,  0.288675,  3.333333,  0.000000,  0.000000, -0.166667 },
	{ -0.166667, -0.288675, -3.333333,  0.000000,  0.000000, -0.166667 },
	{ -0.166667,  0.288675, -3.333333,  0.000000,  0.000000, -0.166667 },
	{  0.166667, -0.288675,  3.333333,  0.000000,  0.000000, -0.166667 },
};

static constexpr MultirotorMixer::Rotor _config_dodeca_top_cox[] {
	{ -0.333333,  0.000000, -3.333333,  0.000000,  0.000000, -0.166667 },
	{  0.333333, -0.000000,  3.333333,  0.000000,  0.000000, -0.166667 },
	{  0.166667,  0.288675, -3.333333,  0.000000,  0.000000, -0.166667 },
	{ -0.166667, -0.288675,  3.333333,  0.000000,  0.000000, -0.166667 },
	{ -0.166667,  0.288675,  3.333333,  0.000000,  0.000000, -0.166667 },
	{  0.166667, -0.288675, -3.333333,  0.000000,  0.000000, -0.166667 },
};

static constexpr MultirotorMixer::Rotor _config_hex_cox[] {
	{ -0.288684,  0.166667, -3.333333,  0.000000,  0.000000, -0.166667 },
	{ -0.288684,  0.166667,  3.333333,  0.000000,  0.000000, -0.166667 },
	{  0.000000, -0.333333, -3.333333,  0.000000,  0.000000, -0.166667 },
	{  0.000000, -0.333333,  3.333333,  0.000000,  0.000000, -0.166667 },
	{  0.288684,  0.166667, -3.333333,  0.000000,  0.000000, -0.166667 },
	{  0.288684,  0.166667,  3.333333,  0.000000,  0.000000, -0.166667 },
};

static constexpr MultirotorMixer::Rotor _config_hex_plus[] {
	{  0.000000,  0.333333, -3.333333,  0.000000,  0.000000, -0.166667 },
	{ -0.000000, -0.333333,  3.333333,  0.000000,  0.000000, -0.166667 },
	{  0.288675, -0.166667, -3.333333,  0.000000,  0.000000, -0.166667 },
	{ -0.288675,  0.166667,  3.333333,  0.000000,  0.000000, -0.166667 },
	{  0.288675,  0.166667,  3.333333,  0.000000,  0.000000, -0.166667 },
	{ -0.288675, -0.166667, -3.333333,  0.000000,  0.000000, -0.166667 },
};

static constexpr MultirotorMixer::Rotor _config_hex_t[] {
	{ -0.365497,  0.144592, -3.333333,  0.000000,  0.000000, -0.144592 },
	{ -0.365497,  0.144592,  3.333333,  0.000000,  0.000000, -0.144592 },
	{ -0.000000, -0.289184, -3.333333,  0.000000,  0.000000, -0.210816 },
	{ -0.000000, -0.289184,  3.333333,  0.000000,  0.000000, -0.210816 },
	{  0.365497,  0.144592, -3.333333,  0.000000,  0.000000, -0.144592 },
	{  0.365497,  0.144592,  3.333333,  0.000000,  0.000000, -0.144592 },
};

static constexpr MultirotorMixer::Rotor _config_hex_x[] {
	{ -0.333333,  0.000000, -3.333333,  0.000000,  0.000000, -0.166667 },
	{  0.333333, -0.000000,  3.333333,  0.000000,  0.000000, -0.166667 },
	{  0.166667,  0.288675, -3.333333,  0.000000,  0.000000, -0.166667 },
	{ -0.166667, -0.288675,  3.333333,  0.000000,  0.000000, -0.166667 },
	{ -0.166667,  0.288675,  3.333333,  0.000000,  0.000000, -0.166667 },
	{  0.166667, -0.288675, -3.333333,  0.000000,  0.000000, -0.166667 },
};

static constexpr MultirotorMixer::Rotor _config_octa_cox[] {
	{ -0.176777,  0.176777,  2.500000,  0.000000,  0.000000, -0.125000 },
	{  0.176777,  0.176777, -2.500000,  0.000000,  0.000000, -0.125000 },
	{  0.176777, -0.176777,  2.500000,  0.000000,  0.000000, -0.125000 },
	{ -0.176777, -0.176777, -2.500000,  0.000000,  0.000000, -0.125000 },
	{  0.176777,  0.176777,  2.500000,  0.000000,  0.000000, -0.125000 },
	{ -0.176777,  0.176777, -2.500000,  0.000000,  0.000000, -0.125000 },
	{ -0.176777, -0.176777,  2.500000,  0.000000,  0.000000, -0.125000 },
	{  0.176777, -0.176777, -2.500000,  0.000000,  0.000000, -0.125000 },
};

static constexpr MultirotorMixer::Rotor _config_octa_cox_wide[] {
	{ -0.158371,  0.249022,  2.500000,  0.000000,  0.000000, -0.156715 },
	{  0.158371,  0.249022, -2.500000,  0.000000,  0.000000, -0.156715 },
	{  0.132743, -0.249022,  2.500000,  0.000000,  0.000000, -0.093285 },
	{ -0.132743, -0.249022, -2.500000,  0.000000,  0.000000, -0.093285 },
	{  0.158371,  0.249022,  2.500000,  0.000000,  0.000000, -0.156715 },
	{ -0.158371,  0.249022, -2.500000,  0.000000,  0.000000, -0.156715 },
	{ -0.132743, -0.249022,  2.500000,  0.000000,  0.000000, -0.093285 },
	{  0.132743, -0.249022, -2.500000,  0.000000,  0.000000, -0.093285 },
};

static constexpr MultirotorMixer::Rotor _config_octa_plus[] {
	{  0.000000,  0.250000, -2.500000,  0.000000,  0.000000, -0.125000 },
	{  0.000000, -0.250000, -2.500000,  0.000000,  0.000000, -0.125000 },
	{ -0.176777,  0.176777,  2.500000,  0.000000,  0.000000, -0.125000 },
	{ -0.176777, -0.176777,  2.500000,  0.000000,  0.000000, -0.125000 },
	{  0.176777,  0.176777,  2.500000,  0.000000,  0.000000, -0.125000 },
	{  0.176777, -0.176777,  2.500000,  0.000000,  0.000000, -0.125000 },
	{  0.250000,  0.000000, -2.500000,  0.000000,  0.000000, -0.125000 },
	{ -0.250000,  0.000000, -2.500000,  0.000000,  0.000000, -0.125000 },
};

static constexpr MultirotorMixer::Rotor _config_octa_x[] {
	{ -0.095671,  0.230970, -2.500000,  0.000000,  0.000000, -0.125000 },
	{  0.095671, -0.230970, -2.500000,  0.000000,  0.000000, -0.125000 },
	{ -0.230970,  0.095671,  2.500000,  0.000000,  0.000000, -0.125000 },
	{ -0.095671, -0.230970,  2.500000,  0.000000,  0.000000, -0.125000 },
	{  0.095671,  0.230970,  2.500000,  0.000000,  0.000000, -0.125000 },
	{  0.230970, -0.095671,  2.500000,  0.000000,  0.000000, -0.125000 },
	{  0.230970,  0.095671, -2.500000,  0.000000,  0.000000, -0.125000 },
	{ -0.230970, -0.095671, -2.500000,  0.000000,  0.000000, -0.125000 },
};

static constexpr MultirotorMixer::Rotor _config_quad_deadcat[] {
	{ -1.156069,  1.650165, 21.676301,  0.000000,  0.000000, -0.309406 },
	{  1.156069, -1.650165, 28.323699,  0.000000,  0.000000, -0.190594 },
	{  1.156069,  1.650165, -21.676301,  0.000000,  0.000000, -0.309406 },
	{ -1.156069, -1.650165, -28.323699,  0.000000,  0.000000, -0.190594 },
};

static constexpr MultirotorMixer::Rotor _config_quad_h[] {
	{ -0.353553,  0.353553, -5.000000,  0.000000,  0.000000, -0.250000 },
	{  0.353553, -0.353553, -5.000000,  0.000000,  0.000000, -0.250000 },
	{  0.353553,  0.353553,  5.000000,  0.000000,  0.000000, -0.250000 },
	{ -0.353553, -0.353553,  5.000000,  0.000000,  0.000000, -0.250000 },
};

static constexpr MultirotorMixer::Rotor _config_quad_plus[] {
	{ -0.500000, -0.000000,  5.000000,  0.000000,  0.000000, -0.250000 },
	{  0.500000, -0.000000,  5.000000,  0.000000,  0.000000, -0.250000 },
	{ -0.000000,  0.500000, -5.000000,  0.000000,  0.000000, -0.250000 },
	{ -0.000000, -0.500000, -5.000000,  0.000000,  0.000000, -0.250000 },
};

static constexpr MultirotorMixer::Rotor _config_quad_s250aq[] {
	{ -2.699784,  2.380952, 14.902808,  0.000000,  0.000000, -0.250000 },
	{  2.699784, -2.380952, 35.097192,  0.000000,  0.000000, -0.250000 },
	{  2.699784,  2.380952, -14.902808,  0.000000,  0.000000, -0.250000 },
	{ -2.699784, -2.380952, -35.097192,  0.000000,  0.000000, -0.250000 },
};

static constexpr MultirotorMixer::Rotor _config_quad_vtail[] {
	{ -2.331288,  1.111111, -0.674847,  0.000000,  0.230061, -0.277778 },
	{  0.057841, -1.571348, -0.231364,  0.000000,  0.614561, -0.314270 },
	{  2.331288,  1.111111,  0.674847,  0.000000, -0.230061, -0.277778 },
	{ -0.057841, -1.571348,  0.231364,  0.000000, -0.614561, -0.314270 },
};

static constexpr MultirotorMixer::Rotor _config_quad_wide[] {
	{ -1.156069,  1.650165,  4.335260,  0.000000,  0.000000, -0.250000 },
	{  1.156069, -1.650165,  5.664740,  0.000000,  0.000000, -0.250000 },
	{  1.156069,  1.650165, -4.335260,  0.000000,  0.000000, -0.250000 },
	{ -1.156069, -1.650165, -5.664740,  0.000000,  0.000000, -0.250000 },
};

static constexpr MultirotorMixer::Rotor _config_quad_x_cw[] {
	{ -0.353553,  0.353553,  5.000000,  0.000000,  0.000000, -0.250000 },
	{ -0.353553, -0.353553, -5.000000,  0.000000,  0.000000, -0.250000 },
	{  0.353553, -0.353553,  5.000000,  0.000000,  0.000000, -0.250000 },
	{  0.353553,  0.353553, -5.000000,  0.000000,  0.000000, -0.250000 },
};

static constexpr MultirotorMixer::Rotor _config_quad_x[] {
	{ -0.353553,  0.353553,  5.000000,  0.000000,  0.000000, -0.250000 },
	{  0.353553, -0.353553,  5.000000,  0.000000,  0.000000, -0.250000 },
	{  0.353553,  0.353553, -5.000000,  0.000000,  0.000000, -0.250000 },
	{ -0.353553, -0.353553, -5.000000,  0.000000,  0.000000, -0.250000 },
};

static constexpr MultirotorMixer::Rotor _config_quad_x_pusher[] {
	{ -0.250000,  0.250000,  5.000000,  0.000000,  0.000000, -0.250000 },
	{  0.250000, -0.250000,  5.000000,  0.000000,  0.000000, -0.250000 },
	{  0.250000,  0.250000, -5.000000,  0.000000,  0.000000, -0.250000 },
	{ -0.250000, -0.250000, -5.000000,  0.000000,  0.000000, -0.250000 },
	{  0.000000,  0.000000,  0.000000,  0.500000,  0.000000,  0.000000 },
};

static constexpr MultirotorMixer::Rotor _config_quad_y[] {
	{ -2.500000,  1.250000, -0.000000,  0.000000,  0.000000, -0.250000 },
	{  2.500000, -1.250000, 10.000000,  0.000000,  0.000000, -0.250000 },
	{  2.500000,  1.250000,  0.000000,  0.000000,  0.000000, -0.250000 },
	{ -2.500000, -1.250000, -10.000000,  0.000000,  0.000000, -0.250000 },
};

static constexpr MultirotorMixer::Rotor _config_tri_y[] {
	{ -0.577351,  0.333333,  0.000000,  0.000000,  0.000000, -0.333333 },
	{  0.577351,  0.333333,  0.000000,  0.000000,  0.000000, -0.333333 },
	{  0.000000, -0.666667,  0.000000,  0.000000,  0.000000, -0.333333 },
};

static constexpr MultirotorMixer::Rotor _config_twin_engine[] {
	{ -0.500000,  0.000000,  0.000000,  0.000000,  0.000000, -0.500000 },
	{  0.500000,  0.000000,  0.000000,  0.000000,  0.000000, -0.500000 },
};

static constexpr const MultirotorMixer::Rotor *_config_index[] {
	&_config_dodeca_bottom_cox[0],
	&_config_dodeca_top_cox[0],
	&_config_hex_cox[0],
	&_config_hex_plus[0],
	&_config_hex_t[0],
	&_config_hex_x[0],
	&_config_octa_cox[0],
	&_config_octa_cox_wide[0],
	&_config_octa_plus[0],
	&_config_octa_x[0],
	&_config_quad_deadcat[0],
	&_config_quad_h[0],
	&_config_quad_plus[0],
	&_config_quad_s250aq[0],
	&_config_quad_vtail[0],
	&_config_quad_wide[0],
	&_config_quad_x_cw[0],
	&_config_quad_x[0],
	&_config_quad_x_pusher[0],
	&_config_quad_y[0],
	&_config_tri_y[0],
	&_config_twin_engine[0],
};

static constexpr unsigned _config_rotor_count[] {
	6, /* dodeca_bottom_cox */
	6, /* dodeca_top_cox */
	6, /* hex_cox */
	6, /* hex_plus */
	6, /* hex_t */
	6, /* hex_x */
	8, /* octa_cox */
	8, /* octa_cox_wide */
	8, /* octa_plus */
	8, /* octa_x */
	4, /* quad_deadcat */
	4, /* quad_h */
	4, /* quad_plus */
	4, /* quad_s250aq */
	4, /* quad_vtail */
	4, /* quad_wide */
	4, /* quad_x_cw */
	4, /* quad_x */
	5, /* quad_x_pusher */
	4, /* quad_y */
	3, /* tri_y */
	2, /* twin_engine */
};

const char* _config_key[] {
	"6a",	/* dodeca_bottom_cox */
	"6m",	/* dodeca_top_cox */
	"6c",	/* hex_cox */
	"6+",	/* hex_plus */
	"6t",	/* hex_t */
	"6x",	/* hex_x */
	"8c",	/* octa_cox */
	"8cw",	/* octa_cox_wide */
	"8+",	/* octa_plus */
	"8x",	/* octa_x */
	"4dc",	/* quad_deadcat */
	"4h",	/* quad_h */
	"4+",	/* quad_plus */
	"4s",	/* quad_s250aq */
	"4vt",	/* quad_vtail */
	"4w",	/* quad_wide */
	"4xcw",	/* quad_x_cw */
	"4x",	/* quad_x */
	"4x1p",	/* quad_x_pusher */
	"4y",	/* quad_y */
	"3y",	/* tri_y */
	"2-",	/* twin_engine */
};

} // anonymous namespace

#endif /* _MIXER_MULTI_TABLES */

