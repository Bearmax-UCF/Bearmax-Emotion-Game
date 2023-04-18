'''
Handles communication with outside system and operates
all nodes involved in the emotion recognition game
'''

#!/usr/bin/env python
import time
from random import randint
from datetime import datetime
from dataclasses import dataclass
from typing import Callable
import json

import bearmax_emotion.emotion_lib.src.utils as utils
ALL_EMOTIONS = utils.ALL_EMOTIONS

NEWROUND_PAUSE = 2  # sec

@dataclass
class Scores:
    happy: int = 0
    sad: int = 0
    angry: int = 0
    neutral: int = 0
    other: int = 0
    invalid: int = 0

    def increment_score(self, name: str):
        if name not in vars(self).keys():
            return
        if not isinstance(getattr(self, name), int):
            return

        setattr(self, name, getattr(self, name) + 1)

    def to_list(self):
        return [self.happy, self.sad, self.angry, self.neutral]
    
    def sum(self):
        return self.happy + self.sad + self.angry + self.neutral


@dataclass
class State:
    started: bool = False
    paused: bool = True
    current_emotion: str = "other"
    target_emotion: str = None
    correct: Scores = Scores()
    wrong: Scores = Scores()


@dataclass
class FinalScore:
    finish_time: datetime
    correct: Scores
    wrong: Scores
    UserID: str

    def to_json_str(self):
        return json.dumps({
            "GameFin": str(self.finish_time),
            "NumPlays": self.correct.sum() + self.wrong.sum(),
            "Correct:": self.correct.to_list(),
            "Wrong": self.wrong.to_list(),
            "UserID": self.UserID
        })


class EmotionGame:
    '''
    In storage arrays:
    0 = Happy
    1 = Sad
    2 = Angry
    3 = Neutral
    4 = Other
    '''

    def __init__(self, logger, send_to_stack):
        self.logger = logger  # The logger from ros node
        self._callbacks = {"new_round": [], "on_win": [], "on_lose": []}
        self.send_to_stack = send_to_stack
        self._pause_start = time.time()
        self._state = State()
        self.round_in_progress = False
        self.UserID = None

    @ property
    def state(self):
        return self._state

    def start(self, UserID: str):
        # Should always start with a clean state
        self.UserID = UserID
        self._state = State()
        self._state.started = True
        self._state.paused = False
        self._chooseNewTargetEmotion()

    def pause(self):
        self._state.paused = True
        self._pause_start = time.time()

    def resume(self):
        self._state.paused = False

    def end(self) -> FinalScore:
        self._state.started = False
        return FinalScore(
            datetime.now(),
            self._state.correct,
            self._state.wrong,
            self.UserID or "",
        )

    def registerCallback(self, event: str, cb: Callable):
        """ Registers a callback function for a specified event
            @param event - One of: 'new_round', 'on_win', 'on_lose'
            @param cb - Callback function
        """
        self._callbacks[event].append(cb)

    def handleEmotionChange(self, emotion: str):
        """
            Called by ROS Node after the detected emotion
            has been held for a sufficient amount of time.
        """
        if not self._state.started:
            return
        if not self.round_in_progress:
            return

        if self._state.paused:
            if self._pause_start <= time.time() - NEWROUND_PAUSE:
                self.resume()
            else:
                return

        self._state.current_emotion = emotion

        if emotion == self._state.target_emotion:
            self._roundWinRoutine()
            self.round_in_progress = False
        else:
            self._roundLoseRoutine()
            self.round_in_progress = False

    def _pushEvent(self, event: str, *args, **kwargs):
        """ Runs the registered callback functions for an event.
            @param event - One of: 'new_round', 'on_win', 'on_lose'
        """
        if not self._state.started or self._state.paused:
            return  # Don't push events when game is not active

        for cb in self._callbacks.get(event, []):
            cb(*args, **kwargs)

    def _newRound(self):
        def cb(*_, **__):
            self.round_in_progress = True
        self._pushEvent("new_round", new_emotion=self._state.target_emotion, round_cb=cb)

    def _roundWinRoutine(self):
        self._state.correct.increment_score(self._state.target_emotion)
        def cb(*_, **__):
            self._chooseNewTargetEmotion()
        self._pushEvent("on_win", cb=cb)

    def _roundLoseRoutine(self):
        self._state.wrong.increment_score(self._state.target_emotion)
        def cb(*_, **__):
            self._chooseNewTargetEmotion()
        self._pushEvent("on_lose",
                        detected_emotion=self._state.current_emotion,
                        target_emotion=self._state.target_emotion,
                        cb=cb)

    def _chooseNewTargetEmotion(self):
        prev = self._state.target_emotion
        nextTarget = prev

        while nextTarget == prev:
            nextTarget = ALL_EMOTIONS[randint(0, len(ALL_EMOTIONS) - 1)]

        self._state.target_emotion = nextTarget
        self._newRound()
