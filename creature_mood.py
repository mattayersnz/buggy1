"""
Mood system for creature-like behavior.

This module implements a dynamic mood system that affects the buggy's
personality, movement patterns, and reactions to stimuli.
"""

import time
import random
from typing import Tuple, Dict, List
from enum import Enum


class Mood(Enum):
    """Mood states for the creature."""
    CALM = "calm"
    CURIOUS = "curious"
    EXCITED = "excited"
    CAUTIOUS = "cautious"


class SoundPattern:
    """Sound pattern configuration for mood-based sounds."""

    def __init__(
        self,
        base_frequency: int,
        frequency_variation: int,
        beep_count: int,
        beep_duration: float,
        beep_gap: float
    ):
        """
        Initialize sound pattern.

        Args:
            base_frequency: Base frequency in Hz
            frequency_variation: Random variation range in Hz
            beep_count: Number of beeps in pattern
            beep_duration: Duration of each beep in seconds
            beep_gap: Gap between beeps in seconds
        """
        self.base_frequency: int = base_frequency
        self.frequency_variation: int = frequency_variation
        self.beep_count: int = beep_count
        self.beep_duration: float = beep_duration
        self.beep_gap: float = beep_gap


class MoodConfig:
    """Configuration for a specific mood state."""

    def __init__(
        self,
        mood_type: Mood,
        led_color: Tuple[int, int, int],
        led_breathing: bool,
        speed_range: Tuple[int, int],
        turn_bias: float,
        pause_frequency: float,
        pause_duration: Tuple[float, float],
        sound_pattern: SoundPattern,
        investigation_distance: float
    ):
        """
        Initialize mood configuration.

        Args:
            mood_type: The mood this config represents
            led_color: RGB tuple (0-255 each)
            led_breathing: Enable LED breathing effect during idle
            speed_range: (min_speed, max_speed) for movement
            turn_bias: Probability of random turns (0.0-1.0)
            pause_frequency: Number of pauses per minute
            pause_duration: (min_seconds, max_seconds) for pauses
            sound_pattern: Sound pattern for this mood
            investigation_distance: How close to approach objects (cm)
        """
        self.mood_type: Mood = mood_type
        self.led_color: Tuple[int, int, int] = led_color
        self.led_breathing: bool = led_breathing
        self.speed_range: Tuple[int, int] = speed_range
        self.turn_bias: float = turn_bias
        self.pause_frequency: float = pause_frequency
        self.pause_duration: Tuple[float, float] = pause_duration
        self.sound_pattern: SoundPattern = sound_pattern
        self.investigation_distance: float = investigation_distance


# Define mood configurations
MOOD_CONFIGS: Dict[Mood, MoodConfig] = {
    Mood.CALM: MoodConfig(
        mood_type=Mood.CALM,
        led_color=(0, 0, 255),  # Blue
        led_breathing=True,
        speed_range=(20, 30),
        turn_bias=0.2,
        pause_frequency=3.0,  # 3 pauses per minute
        pause_duration=(1.0, 2.0),
        sound_pattern=SoundPattern(400, 50, 1, 0.15, 0.1),
        investigation_distance=8.0
    ),
    Mood.CURIOUS: MoodConfig(
        mood_type=Mood.CURIOUS,
        led_color=(0, 255, 255),  # Cyan
        led_breathing=True,
        speed_range=(25, 40),
        turn_bias=0.35,
        pause_frequency=5.0,  # 5 pauses per minute
        pause_duration=(0.5, 1.5),
        sound_pattern=SoundPattern(600, 100, 3, 0.1, 0.08),
        investigation_distance=5.0
    ),
    Mood.EXCITED: MoodConfig(
        mood_type=Mood.EXCITED,
        led_color=(255, 150, 0),  # Yellow/Orange
        led_breathing=False,
        speed_range=(35, 60),
        turn_bias=0.5,
        pause_frequency=1.5,  # 1.5 pauses per minute
        pause_duration=(0.3, 0.8),
        sound_pattern=SoundPattern(900, 200, 5, 0.08, 0.05),
        investigation_distance=10.0
    ),
    Mood.CAUTIOUS: MoodConfig(
        mood_type=Mood.CAUTIOUS,
        led_color=(180, 0, 255),  # Purple
        led_breathing=True,
        speed_range=(15, 25),
        turn_bias=0.4,
        pause_frequency=6.5,  # 6.5 pauses per minute
        pause_duration=(2.0, 3.0),
        sound_pattern=SoundPattern(350, 30, 2, 0.25, 0.15),
        investigation_distance=12.0
    )
}


class MoodManager:
    """
    Manages mood state transitions and event tracking.

    Moods change based on random timers and environmental events
    (obstacles, boundaries, etc.).
    """

    # Transition probabilities: MOOD_TRANSITIONS[from_mood][to_mood] = probability
    MOOD_TRANSITIONS: Dict[Mood, Dict[Mood, float]] = {
        Mood.CALM: {
            Mood.CALM: 0.40,
            Mood.CURIOUS: 0.30,
            Mood.EXCITED: 0.20,
            Mood.CAUTIOUS: 0.10
        },
        Mood.CURIOUS: {
            Mood.CALM: 0.20,
            Mood.CURIOUS: 0.35,
            Mood.EXCITED: 0.30,
            Mood.CAUTIOUS: 0.15
        },
        Mood.EXCITED: {
            Mood.CALM: 0.25,
            Mood.CURIOUS: 0.25,
            Mood.EXCITED: 0.40,
            Mood.CAUTIOUS: 0.10
        },
        Mood.CAUTIOUS: {
            Mood.CALM: 0.40,
            Mood.CURIOUS: 0.30,
            Mood.EXCITED: 0.05,
            Mood.CAUTIOUS: 0.25
        }
    }

    # Mood duration range (seconds)
    MOOD_DURATION_MIN: float = 15.0
    MOOD_DURATION_MAX: float = 45.0

    def __init__(self, start_mood: Mood = Mood.CURIOUS):
        """
        Initialize mood manager.

        Args:
            start_mood: Initial mood state
        """
        self.current_mood: Mood = start_mood
        self.mood_start_time: float = time.time()
        self.mood_duration: float = random.uniform(
            self.MOOD_DURATION_MIN,
            self.MOOD_DURATION_MAX
        )
        self.event_counts: Dict[str, int] = {
            "object_encounter": 0,
            "boundary_encounter": 0,
            "idle_time": 0
        }

    def update(self) -> MoodConfig:
        """
        Update mood state and check for transitions.

        Returns:
            Current mood configuration
        """
        current_time = time.time()
        time_in_mood = current_time - self.mood_start_time

        # Check if it's time for a mood change
        if time_in_mood >= self.mood_duration:
            self._transition_mood()

        return MOOD_CONFIGS[self.current_mood]

    def _transition_mood(self) -> None:
        """Execute a mood transition based on probabilities."""
        # Get transition probabilities for current mood
        transitions = self.MOOD_TRANSITIONS[self.current_mood]

        # Choose new mood based on weighted probabilities
        moods = list(transitions.keys())
        probabilities = [transitions[mood] for mood in moods]

        new_mood = random.choices(moods, weights=probabilities, k=1)[0]

        # Update mood state
        self.current_mood = new_mood
        self.mood_start_time = time.time()
        self.mood_duration = random.uniform(
            self.MOOD_DURATION_MIN,
            self.MOOD_DURATION_MAX
        )

    def log_event(self, event_type: str) -> None:
        """
        Log an event and potentially trigger mood change.

        Args:
            event_type: Type of event ("object_encounter", "boundary_encounter", "idle_time")
        """
        if event_type in self.event_counts:
            self.event_counts[event_type] += 1

        # Event-triggered mood changes (15% of transitions for obstacles/boundaries)
        if event_type == "boundary_encounter" and random.random() < 0.3:
            # Boundaries might make creature cautious
            if random.random() < 0.6:
                self._force_mood(Mood.CAUTIOUS)
            else:
                self._transition_mood()

        elif event_type == "object_encounter" and random.random() < 0.2:
            # Objects might make creature curious or cautious
            if random.random() < 0.7:
                self._force_mood(Mood.CURIOUS)
            else:
                self._force_mood(Mood.CAUTIOUS)

    def _force_mood(self, new_mood: Mood) -> None:
        """
        Force a specific mood change.

        Args:
            new_mood: Mood to change to
        """
        if new_mood != self.current_mood:
            self.current_mood = new_mood
            self.mood_start_time = time.time()
            self.mood_duration = random.uniform(
                self.MOOD_DURATION_MIN,
                self.MOOD_DURATION_MAX
            )

    def get_current_config(self) -> MoodConfig:
        """
        Get current mood configuration.

        Returns:
            Current MoodConfig
        """
        return MOOD_CONFIGS[self.current_mood]

    def should_pause(self) -> bool:
        """
        Check if creature should pause based on current mood.

        Returns:
            True if should pause, False otherwise
        """
        config = MOOD_CONFIGS[self.current_mood]

        # Convert pauses per minute to probability per decision cycle
        # Assuming ~10 decision cycles per second (0.1s loop delay)
        pauses_per_second = config.pause_frequency / 60.0
        pause_probability = pauses_per_second * 0.1  # Per 0.1s cycle

        return random.random() < pause_probability

    def get_pause_duration(self) -> float:
        """
        Get random pause duration for current mood.

        Returns:
            Pause duration in seconds
        """
        config = MOOD_CONFIGS[self.current_mood]
        return random.uniform(*config.pause_duration)

    def get_random_speed(self) -> int:
        """
        Get random speed within current mood's range.

        Returns:
            Speed value (0-100)
        """
        config = MOOD_CONFIGS[self.current_mood]
        return random.randint(*config.speed_range)
