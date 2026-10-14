# Privacy

This robot is meant to work in older people's homes. Some of what it and its
tools record shows the inside of those homes, and this repository is
**public**: anything committed here can be read, copied and kept by anyone.

## What is sensitive

- **Phone scans (Polycam `.glb`).** These carry a photo texture of every
  surface: the layout, and whatever was lying around — medicine, letters,
  photos, screens, faces.
- **Recordings** from `robot_telemetry` and the mapping launches (`.mcap`
  bags), once a camera or microphone is added.
- **Voice** audio and transcripts from `robot_voice`. The consent conditions
  for those are in [decision 0002](../decisions/0002-voice-provider.md).

Maps (`.pgm`/`.yaml`) and the obstacle report (`room.json`) hold shapes and
sizes only. They still show a room's layout, so treat a real home's map as
personal too, but they carry no photos.

## Rules

1. **Ask first.** Before scanning or recording in anyone else's home, tell
   them what is captured, where it goes and who can see it, and get a yes.
2. **Scans never go into git.** `.gitignore` excludes `docs/runs/*.glb`. The
   one exception, `docs/runs/room-scan-2026-10-05.glb`, is a test patch of bare floor
   and balls that the tests need.
3. **Keep scans on your own machine** or in private storage you control.
4. **Check who can open the map web page.** Anyone it's shared with sees the
   3D scan too. Sharing is set in the page's Share menu.
5. **If something sensitive is pushed by mistake,** tell the other people on
   the repository first. Removing it means rewriting git history, which
   breaks everyone's copy, and it may already have been copied, so treat it
   as exposed either way.
