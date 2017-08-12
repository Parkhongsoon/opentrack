#include "logic.hpp"

using namespace calibrator_wizard;

constexpr tuple Wizard::actions[];

tuple Wizard::pose_diff(const tuple& pose) const
{
    if (is_done())
        return tuple(y_nada, p_nada);

    yaw_action ret_y = y_nada, pose_y, cur_yaw;
    pitch_action ret_p = p_nada, pose_p, cur_pitch;

    std::tie(pose_y, pose_p) = pose;
    std::tie(cur_yaw, cur_pitch) = expected();

    if (cur_yaw != pose_y)
        ret_y = pose_y;

    if (cur_pitch != pose_p)
        ret_p = pose_p;

    return tuple(ret_y, ret_p);
}

bool Wizard::maybe_next(const pose_t& pose)
{
    if (is_done())
        return false;

    tuple expected_ = expected();
    tuple current_ = cur_pose(pose);

    const bool ret = current_ == expected_;

    static constexpr double d2r = M_PI/180;

    rmat R_ = euler::euler_to_rmat(d2r * euler_angles_t(&pose[Yaw]));
    cv::Matx33d R;

    for (unsigned i = 0; i < 3; i++)
        for (unsigned j = 0; j < 3; j++)
            R(i, j) = R_(i, j);

    cv::Vec3d T(pose[TX], pose[TY], pose[TZ]);

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

        if (expected_y != current_y)
            yaw_reached = false;
        if (expected_p != current_p)
            pitch_reached = false;

        reached_p = bb(yaw_reached, pitch_reached);
    }

    return ret;
}

bool Wizard::is_done() const
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

tuple Wizard::expected() const
{
    if (iter == std::end(actions))
        return tuple(y_nada, p_nada);

    return *iter;
}

tuple Wizard::cur_pose(const pose_t& pose) const
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
    if (pose(Yaw) > yaw_side)
        ret_y = y_right;
    else if (pose(Yaw) < -yaw_side)
        ret_y = y_left;

    // up is plus, down is minus
    if (pose(Pitch) > pitch_up)
        ret_p = p_up;
    else if (pose(Pitch) < pitch_down)
        ret_p = p_down;

    return tuple(ret_y, ret_p);
}

QString Wizard::action_to_string(const tuple& pose)
{
    return QString();
}

Wizard::Wizard(QObject* parent) :
    QObject(parent),
    calibrator(1, 2, 0),
    iter(std::begin(actions)),
    reached_p(false, false)
{
}
