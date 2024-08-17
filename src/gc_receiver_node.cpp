#include "oxebots_comms/gc_receiver_node.hpp"

GCReceiverNode::GCReceiverNode(boost::asio::io_context & io_context)
: rclcpp::Node("oxebots_comms"),
  UdpDriver<Referee>(io_context),
  io_context(io_context)
{
    declare_parameter("host", "224.5.23.1");
    declare_parameter("port", 10003);
    declare_parameter("topic_retention", 10);
    declare_parameter("is_team_info", false);
    declare_parameter("gc_topic", "gc_data");

    RCLCPP_DEBUG(get_logger(), "Creating host...");

    add_host(get_parameter("host").as_string(),
             get_parameter("port").as_int());

    RCLCPP_DEBUG(get_logger(), "Creating GC Publisher...");

    io_thread = std::thread([this]() { this->io_context.run(); });

    RCLCPP_DEBUG(get_logger(), "Creating gc publisher...");

    gc_publisher = create_publisher<oxebots_interfaces::msg::Referee>(
      get_parameter("gc_topic").as_string(),
      get_parameter("topic_retention").as_int());

    RCLCPP_INFO(get_logger(), "Game controller receiver module started");
}

GCReceiverNode::~GCReceiverNode()
{
    RCLCPP_INFO(get_logger(), "Stopping game receiver module...");
    io_context.stop();
    if (io_thread.joinable()) io_thread.join();
    stop();
}

oxebots_interfaces::msg::TeamInfo GCReceiverNode::get_team_info(
  const Referee_TeamInfo & team)
{
    oxebots_interfaces::msg::TeamInfo team_info;

    team_info.name = team.name();
    team_info.score = team.score();
    team_info.red_cards = team.red_cards();
    team_info.yellow_cards = team.yellow_cards();
    team_info.timeouts = team.timeouts();
    team_info.timeout_time = team.timeout_time();
    team_info.goalkeeper = team.goalkeeper();

    if (team.has_foul_counter())
        team_info.foul_counter = team.foul_counter();
    else
        team_info.foul_counter = 0;

    if (team.has_ball_placement_failures())
        team_info.ball_placement_failures = team.ball_placement_failures();
    else
        team_info.ball_placement_failures = 0;

    if (team.has_can_place_ball())
        team_info.can_place_ball = team.can_place_ball();
    else
        team_info.can_place_ball = false;

    if (team.has_max_allowed_bots())
        team_info.max_allowed_bots = team.max_allowed_bots();
    else
        team_info.max_allowed_bots = 0;

    if (team.has_bot_substitution_intent())
        team_info.bot_substitution_intent = team.bot_substitution_intent();
    else
        team_info.bot_substitution_intent = false;

    if (team.has_ball_placement_failures_reached())
        team_info.ball_placement_failures_reached =
          team.ball_placement_failures_reached();
    else
        team_info.ball_placement_failures_reached = false;

    if (team.has_bot_substitution_allowed())
        team_info.bot_substitution_allowed = team.bot_substitution_allowed();
    else
        team_info.bot_substitution_allowed = false;

    if (team.has_bot_substitutions_left())
        team_info.bot_substitutions_left = team.bot_substitutions_left();
    else
        team_info.bot_substitutions_left = 0;

    if (team.has_bot_substitution_time_left())
        team_info.bot_substitution_time_left =
          team.bot_substitution_time_left();
    else
        team_info.bot_substitution_time_left = 0;

    return team_info;
}

void GCReceiverNode::get_ball_left_field(
  const GameEvent_BallLeftField & ball_left_field,
  oxebots_interfaces::msg::GameEvent & game_event)
{
    game_event.by_robot = ball_left_field.by_bot();
    auto location = oxebots_interfaces::msg::Vector2();
    location.x = ball_left_field.location().x();
    location.y = ball_left_field.location().y();
    game_event.location = location;
}

oxebots_interfaces::msg::GameEvent GCReceiverNode::get_game_event(
  const GameEvent & event)
{
    oxebots_interfaces::msg::GameEvent game_event;
    game_event.id = event.id();
    for (auto origin : event.origin()) game_event.origin.push_back(origin);
    game_event.created_timestamp = event.created_timestamp();

    game_event.event_type = event.event_case();
    game_event.by_team = event.aimless_kick().by_team();
    switch (event.event_case())
    {
        case oxebots_interfaces::msg::GameEvent::BALL_LEFT_FIELD_GOAL_LINE:
        {
            get_ball_left_field(event.ball_left_field_goal_line(), game_event);
            break;
        }
        case oxebots_interfaces::msg::GameEvent::BALL_LEFT_FIELD_TOUCH_LINE:
        {
            get_ball_left_field(event.ball_left_field_touch_line(),
                                game_event);
            break;
        }
        case oxebots_interfaces::msg::GameEvent::AIMLESS_KICK:
        {
            game_event.by_robot = event.aimless_kick().by_bot();
            auto location = oxebots_interfaces::msg::Vector2();
            location.x = event.aimless_kick().location().x();
            location.y = event.aimless_kick().location().y();
            game_event.location = location;
            auto kick_location = oxebots_interfaces::msg::Vector2();
            kick_location.x = event.aimless_kick().kick_location().x();
            kick_location.y = event.aimless_kick().kick_location().y();
            game_event.kick_location = kick_location;
            break;
        }
        case oxebots_interfaces::msg::GameEvent::
          ATTACKER_TOO_CLOSE_TO_DEFENSE_AREA:
        {
            game_event.by_robot =
              event.attacker_too_close_to_defense_area().by_bot();
            auto location = oxebots_interfaces::msg::Vector2();
            location.x =
              event.attacker_too_close_to_defense_area().location().x();
            location.y =
              event.attacker_too_close_to_defense_area().location().y();
            game_event.location = location;
            break;
        }
        case oxebots_interfaces::msg::GameEvent::PLACEMENT_SUCCEEDED:
        {
            if (event.placement_succeeded().has_time_taken())
                game_event.time =
                  rclcpp::Time(event.placement_succeeded().time_taken());

            if (event.placement_succeeded().has_precision())
                game_event.precision = event.placement_succeeded().precision();

            if (event.placement_succeeded().has_distance())
                game_event.distance = event.placement_succeeded().distance();

            break;
        }
        case oxebots_interfaces::msg::GameEvent::PENALTY_KICK_FAILED:
        {
            if (event.penalty_kick_failed().has_reason())
                game_event.msg = event.penalty_kick_failed().reason();

            if (event.penalty_kick_failed().has_location())
            {
                auto location = oxebots_interfaces::msg::Vector2();
                location.x = event.penalty_kick_failed().location().x();
                location.y = event.penalty_kick_failed().location().y();
                game_event.location = location;
            }
            break;
        }
        case oxebots_interfaces::msg::GameEvent::NO_PROGRESS_IN_GAME:
        {
            if (event.no_progress_in_game().has_location())
            {
                auto location = oxebots_interfaces::msg::Vector2();
                location.x = event.no_progress_in_game().location().x();
                location.y = event.no_progress_in_game().location().y();
                game_event.location = location;
            }
            if (event.no_progress_in_game().has_time())
                game_event.time =
                  rclcpp::Time(event.no_progress_in_game().time());
            break;
        }
        case oxebots_interfaces::msg::GameEvent::PLACEMENT_FAILED:
        {
            if (event.placement_failed().has_remaining_distance())
                game_event.distance =
                  event.placement_failed().remaining_distance();

            if (event.placement_failed().has_nearest_own_bot_distance())
                game_event.nearest_bot_distance =
                  event.placement_failed().nearest_own_bot_distance();

            break;
        }
        case oxebots_interfaces::msg::GameEvent::CHALLENGE_FLAG_HANDLED:
        {
            game_event.accepted = event.challenge_flag_handled().accepted();
            break;
        }
        default:
        {
            break;
        }
    }

    return game_event;
}

void GCReceiverNode::on_receive(const Referee & packet)
{
    RCLCPP_INFO(get_logger(), "%s", packet.DebugString().c_str());
    oxebots_interfaces::msg::Referee gc_referee;
    gc_referee.source_identifier = packet.source_identifier();
    gc_referee.match_type = packet.match_type();
    // Convert uint64 to builtin_interfaces/Time
    gc_referee.timestamp = rclcpp::Time(packet.packet_timestamp());
    gc_referee.stage = packet.stage();
    gc_referee.stage_time_left = rclcpp::Time(packet.stage_time_left());
    gc_referee.command = packet.command();
    gc_referee.command_counter = packet.command_counter();
    gc_referee.command_timestamp = rclcpp::Time(packet.command_timestamp());
    gc_referee.yellow = get_team_info(packet.yellow());
    gc_referee.blue = get_team_info(packet.blue());
    geometry_msgs::msg::Point32 designated_position;
    designated_position.x = packet.designated_position().x();
    designated_position.y = packet.designated_position().y();
    gc_referee.designated_position = designated_position;

    if (packet.has_blue_team_on_positive_half())
        gc_referee.blue_team_on_positive_half =
          packet.blue_team_on_positive_half();
    else
        gc_referee.blue_team_on_positive_half = false;

    if (packet.has_next_command())
        gc_referee.next_command = packet.next_command();
    else
        gc_referee.next_command = 0;

    gc_referee.game_events = std::vector<oxebots_interfaces::msg::GameEvent>();
    for (auto event : packet.game_events())
        gc_referee.game_events.push_back(get_game_event(event));

    gc_referee.game_event_proposals =
      std::vector<oxebots_interfaces::msg::GameEventProposalGroup>();

    for (auto proposal : packet.game_event_proposals())
    {
        oxebots_interfaces::msg::GameEventProposalGroup proposal_group;
        proposal_group.id = proposal.id();
        for (auto event : proposal.game_events())
            proposal_group.game_events.push_back(get_game_event(event));

        proposal_group.accepted = proposal.accepted();
        gc_referee.game_event_proposals.push_back(proposal_group);
    }

    if (packet.has_current_action_time_remaining())
        gc_referee.current_action_time_remaining =
          rclcpp::Time(packet.current_action_time_remaining());
    else
        gc_referee.current_action_time_remaining = rclcpp::Time(0);

    if (packet.has_status_message())
        gc_referee.status_message = packet.status_message();
    else
        gc_referee.status_message = "";

    PublishGCData(gc_referee);
}

void GCReceiverNode::PublishGCData(oxebots_interfaces::msg::Referee gc_referee)
{
    gc_publisher->publish(gc_referee);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    boost::asio::io_context io_context;

    rclcpp::spin(std::make_shared<GCReceiverNode>(io_context));
    rclcpp::shutdown();
    return 0;
}
