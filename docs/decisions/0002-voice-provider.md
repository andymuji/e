# 0002: Speech recognition provider

- Status: Accepted (the engine choice is deferred; the seam is not)
- Date: 2026-09-20

## Context

`robot_voice` already parses transcripts, refuses destinations the operator has not approved, and turns approved ones into Nav2 goals. What it never had is a source of transcripts: nothing in this repository published `speech_transcript`, so the voice path could not be demonstrated or tested end to end.

Choosing a speech engine well needs facts we do not have yet: how this resident actually speaks, how the room sounds, how much CPU the robot's computer has left while Nav2 is running, and how the household feels about audio of their home leaving it. Picking an engine now would be guessing, and the guess would be expensive to undo once a model, a microphone, and a vendor contract are wired in.

## Decision

Defer the engine, and commit to the seam instead.

`robot_voice.recognizer.SpeechRecognizer` defines what any engine may do: yield final transcripts, and nothing else. `TypedTextRecognizer` implements it with typed text, and `ros2 run robot_voice say "go to the kitchen"` publishes that text on `speech_transcript`. Typed text is a transcript like any other, so the whole path — parse, refuse, goal, stop — is exercisable today with no microphone and no model download, and is the sanctioned replacement for `ros2 topic pub`, which repository policy denies.

Adding Vosk, whisper.cpp, or a cloud service later is an adapter behind that seam, not a rewrite, and an adapter cannot widen what the robot will do: it produces text, and the text still faces the same parser and the same allow-list.

The options this defers between:

- **On-device, offline** (Vosk small model, whisper.cpp). No audio leaves the house, and it keeps working when the broadband does not. Lower accuracy on quiet, accented, or dysarthric speech, and it spends CPU on the robot's own computer, which Nav2 and the safety gate also need.
- **Cloud** (a large vendor's speech API). The best accuracy, particularly for the speech that offline models handle worst, which is exactly the speech of the people this robot is for. It needs a working network, its latency varies with the link, and it sends recordings of a private home to a third party.
- **Hybrid**: a small offline model listening only for stop words, with a cloud engine for destinations. Accuracy where a mistake is a wasted trip, local reliability where a mistake is a hazard. The most moving parts, and the option to take if the trial below says offline destination accuracy is unusable.

What would settle it, in order:

1. A transcription trial with the actual resident, in the actual room, at the distances the robot works from — scored separately for the approved destination names and for "stop".
2. Measured CPU headroom on the robot's computer with Nav2 and the gate running.
3. The household's answer, in writing, on whether audio of their home may be sent to a vendor.
4. Measured behaviour with the network unplugged mid-command.

Until all four exist, typed text is the engine, and that is enough to keep building and testing everything downstream of it.

## Consequences

- **Recognition quality and network availability must never affect the robot's ability to stop.** No engine may sit between a stop condition and the safety gate. The gate stops on its own when the command stream or the sensor stream lapses, the software emergency stop is latched, and the physical, independently wired emergency stop remains the primary one. A voice engine is a way to ask for a stop, never the thing a stop depends on.
- **A misheard destination fails closed to a refusal, not to a nearby-sounding room.** Destination names are matched exactly against the operator-approved store. No fuzzy, phonetic, or nearest-name matching may be added behind this seam, and no engine's confidence score or alternative hypotheses may be used to widen the allow-list. Transcript clean-up may only remove noise words; it may never supply a word the person did not say.
- **"stop" must survive degraded recognition.** Stop words are matched anywhere in an utterance, including inside a truncated or repeated one, and deliberately including "do not stop": a needless stop is a nuisance, a missed one is a hazard. A stop word the engine mangled — "sto", "hal", or the "wait" and "whoa" people shout at a moving robot — is treated as a stop of its own kind: the trip is cancelled and the wheels stop, but the latch is not engaged. That split is deliberate in both directions. A refusal would have left a moving robot moving, which is the hazard; latching on a misheard syllable would leave the robot stopped until an operator walked over to reset it, and a robot stranded that way cannot fetch anyone either. Whether an utterance nobody can parse should also halt a trip in progress is not settled here; it depends on whether the robot listens continuously, which this record defers. An engine that delays final text — batching an utterance, or needing a round trip before it will emit "stop" — is disqualified unless a local stop-word listener runs alongside it. A cloud engine therefore implies the hybrid option, not the cloud option alone.
- **Nothing in `robot_voice` publishes `emergency_stop_reset`,** and no engine changes that. Releasing the latch stays an operator action at the gate; a voice that can undo a stop is not a stop.
- **Recordings of a person's home are a privacy decision, not only a technical one.** A microphone in an elder-care setting hears visitors, carers, and medical conversation, from people who never agreed to anything. Cloud recognition requires the resident's informed consent, a stated retention limit, a way for them to see and delete what was kept, and a visible indication of when the microphone is live. Without that consent, on-device is the only admissible option no matter what the accuracy trial says.
- Wake word versus push-to-talk, and whether the robot listens continuously at all, ride on this decision and are not settled here.
- Adopting an engine, or changing one, requires a new decision record and a repeat of the four measurements above.
