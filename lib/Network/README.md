# lib/Network — commit the source, do **not** symlink

Arduino core 3.3.12 ships `Network` as framework source with an Arduino IDE
`library.properties` but **no PlatformIO `library.json`**. PlatformIO therefore
will not build or link it on its own. `WiFi` then compiles against headers that
resolve, but the link fails on undefined `NetworkInterface` symbols, and the
error does not point at the real cause.

## Policy for this repo: commit the source

Put the real `Network` library source in `lib/Network/src/` **in this repo**.

Do **not** create a symlink into `~/.platformio/packages/...`. A symlink there is
an absolute path into one machine's tree; it resolves on that machine and
nowhere else, and this repo is shared. The upstream project still uses a
symlink — that is fine for a single host, and wrong for a shared one.

The framework version is pinned by URL in `platformio.ini`
(pioarduino espressif32 55.03.312, Arduino core 3.3.12), so the source is
stable: it changes only when the pin changes, and that is a deliberate act.

## Doing it

The framework tree is at:

```
~/.platformio/packages/framework-arduinoespressif32/libraries/Network/src
```

Copy it in, then drop the authored manifest on top:

```bash
cd lib/Network
rm -f src
cp -r ~/.platformio/packages/framework-arduinoespressif32/libraries/Network/src .
git add src library.json
```

`library.json` is already here and must stay. `srcFilter: "+<*.cpp>"` is what
makes PlatformIO build the sources rather than only the headers.

## If the framework tree is not readable

`Permission denied` on that path means the PlatformIO tree belongs to another
user, which is normal on a shared machine. If that happens to you, the copy
cannot be made from that session. Do **not** paper over it with a symlink,
because that reintroduces exactly the problem this policy exists to avoid.
Say so and let a session that can read the path do it.

## Do not

- **Delete `library.json`.** The link fails without it.
- **Symlink `src` into `~/.platformio/packages/`.** See above.
- **Never write into `~/.platformio/packages/`.** PlatformIO owns that tree and
  may overwrite it.
