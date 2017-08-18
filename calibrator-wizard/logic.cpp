#include "logic.hpp"

using namespace calibrator_wizard;
using namespace calibrator_wizard::detail;

constexpr tuple wizard_base::actions[];

tuple wizard_base::pose_diff() const
{
    if (is_done())
        return tuple(y_nada, p_nada);

    yaw_action cur_y, exp_yaw, ret_y = y_nada;
    pitch_action cur_p, exp_pitch, ret_p = p_nada;

    std::tie(cur_y, cur_p) = cur_pose();
    std::tie(exp_yaw, exp_pitch) = expected();

    if (exp_yaw != cur_y)
        ret_y = cur_y;

    if (exp_pitch != cur_p)
        ret_p = cur_p;

    return tuple(ret_y, ret_p);
}

bool wizard_base::maybe_next()
{
    if (is_done())
        return false;

    get_pose(euler, R, T);

    tuple expected_ = expected();
    tuple current_ = cur_pose();

    const bool ret = current_ == expected_;

    calibrator.update(R, T);

    if (ret)
    {
        reached_p = bb(false, false);
        ++iter;
    }
    else
    {
        yaw_action expected_y, current_y;
        pitch_action expected_p, current_p;
        std::tie(expected_y, expected_p) = expected_;

        bool yaw_reached, pitch_reached;
        std::tie(yaw_reached, pitch_reached) = reached_p;

        yaw_reached = expected_y == current_y;
        pitch_reached = expected_p == current_p;
        reached_p = bb(yaw_reached, pitch_reached);
    }

    return ret;
}

bool wizard_base::is_done() const
{
    if (iter == std::end(actions))
        return false;
}

// `start' is how much the mode needs before triggering
// `sustain' is the minimum for the duration of given mode

// if `sustain' isn't reached anymore, `start' is needed again.
// whether `start' got reached is stored in `{yaw,pitch}_reached'.

static constexpr int pitch_up_start = 20;
static constexpr int pitch_up_sustain = 8;

static constexpr int pitch_down_start = -15;
static constexpr int pitch_down_sustain = -6;

static constexpr int yaw_side_start = 18;
static constexpr int yaw_side_sustain = 10;

tuple wizard_base::expected() const
{
    if (iter == std::end(actions))
        return tuple(y_nada, p_nada);

    return *iter;
}

TranslationCalibrator wizard_base::make_from_calib_order()
{
    calibrator_order order = get_calibrator_order();

    int y, p, r;
    std::tie(y, p, r) = order;

    return TranslationCalibrator(y, p, r);
}

tuple wizard_base::cur_pose() const
{
    yaw_action ret_y = y_center;
    pitch_action ret_p = p_center;

    int pitch_up, pitch_down, yaw_side;

    bool yaw_reached, pitch_reached;
    std::tie(yaw_reached, pitch_reached) = reached_p;

    if (pitch_reached)
        pitch_up = pitch_up_sustain, pitch_down = pitch_down_sustain;
    else
        pitch_up = pitch_up_start, pitch_down = pitch_down_start;

    if (yaw_reached)
        yaw_side = yaw_side_sustain;
    else
        yaw_side = yaw_side_start;

    // left is minus, right is plus
    if (euler(Yaw) > yaw_side)
        ret_y = y_right;
    else if (euler(Yaw) < -yaw_side)
        ret_y = y_left;

    // up is plus, down is minus
    if (euler(Pitch) > pitch_up)
        ret_p = p_up;
    else if (euler(Pitch) < pitch_down)
        ret_p = p_down;

    return tuple(ret_y, ret_p);
}

QString wizard_base::action_to_string()
{
    return QString();
}

wizard_base::wizard_base() :
    calibrator(make_from_calib_order()), // no std::apply in c++14
    iter(std::begin(actions)),
    reached_p(false, false)
{
    R = cv::Matx33d::eye();
    T = cv::Vec3d(0, 0, 0);
    euler = pose_t();
}

wizard_base::~wizard_base()
{
}

// template version

template<typename tracker_type>
void wizard<tracker_type>::get_pose(pose_t& euler, cv::Matx33d& R, cv::Vec3d& T) const
{
    return traits::get_pose(euler, R, T);
}

template<typename tracker_type>
calibrator_order wizard<tracker_type>::get_calibrator_order() const
{
    return traits::get_calibrator_order();
}

template<typename tracker_type>
wizard<tracker_type>::wizard(std::shared_ptr<const tracker_type> tracker)
    : tracker(tracker)
{
}
