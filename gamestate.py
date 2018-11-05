#!/usr/bin/env python
# -*- coding:utf-8 -*-

from construct import Byte, Struct, Enum, Bytes, Const, Array, Renamed, Int16ul

Short = Int16ul

RobotInfo = "robot_info" / Struct(
    # define NONE                        0
    # define PENALTY_HL_KID_BALL_MANIPULATION    1
    # define PENALTY_HL_KID_PHYSICAL_CONTACT     2
    # define PENALTY_HL_KID_ILLEGAL_ATTACK       3
    # define PENALTY_HL_KID_ILLEGAL_DEFENSE      4
    # define PENALTY_HL_KID_REQUEST_FOR_PICKUP   5
    # define PENALTY_HL_KID_REQUEST_FOR_SERVICE  6
    # define PENALTY_HL_KID_REQUEST_FOR_PICKUP_2_SERVICE 7
    # define MANUAL                      15
    "penalty" / Byte,
    "secs_till_unpenalised" / Byte,
    "number_of_yellow_cards" / Byte,
    "number_of_red_cards" / Byte
)

TeamInfo = "team" / Struct(
    "team_number" / Byte,
    "team_color" / Enum(Byte,
                        BLUE=0,
                        RED=1,
                        YELLOW=2,
                        BLACK=3,
                        WHITE=4,
                        GREEN=5,
                        ORANGE=6,
                        PURPLE=7,
                        BROWN=8,
                        GRAY=9
                        ),
    "score" / Byte,
    "penalty_shot" / Byte,  # penalty shot counter
    "single_shots" / Short,  # bits represent penalty shot success
    "coach_sequence" / Byte,
    "coach_message" / Bytes(253),
    Renamed("coach", RobotInfo),
    "players" / Array(11, RobotInfo)
)

GameState = "gamedata" / Struct(
    "header" / Const(Bytes(4), b'RGme'),
    "version" / Const(Short, 12),
    "packet_number" / Byte,
    "players_per_team" / Byte,
    "game_type" / Byte,
    "game_state" / Enum(Byte,
                        STATE_INITIAL=0,
                        # auf startposition gehen
                        STATE_READY=1,
                        # bereithalten
                        STATE_SET=2,
                        # spielen
                        STATE_PLAYING=3,
                        # spiel zu ende
                        STATE_FINISHED=4
                        ),
    "first_half" / Byte,
    "kick_of_team" / Byte,
    "secondary_state" / Enum(Byte,
                             STATE_NORMAL=0,
                             STATE_PENALTYSHOOT=1,
                             STATE_OVERTIME=2,
                             STATE_TIMEOUT=3,
                             STATE_DIRECT_FREEKICK=4,
                             STATE_INDIRECT_FREEKICK=5,
                             STATE_PENALTYKICK=6,
                             DROPBALL=128,
							 UNKNOWN=255
                             ),
    "secondary_state_info" / Bytes(4),
    "drop_in_team" / Byte,
    "drop_in_time" / Short,
    "seconds_remaining" / Short,
    "secondary_seconds_remaining" / Short,
    Array(2, Renamed("teams", TeamInfo))
)

GAME_CONTROLLER_RESPONSE_VERSION = 2

ReturnData = "returndata" / Struct(
    "header" / Const(Bytes(4), b"RGrt"),
    "version" / Const(Byte, 2),
    "team" / Byte,
    "player" / Byte,
    "message" / Byte
)
