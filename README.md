# KM3-rs

> A Rust firmware for the KM3 dual stepper motor controller.

[`probe-run`]: https://crates.io/crates/probe-run
[`defmt`]: https://github.com/knurling-rs/defmt
[`flip-link`]: https://github.com/knurling-rs/flip-link

## Dependencies

#### 1. `flip-link`:

```console
$ cargo install flip-link
```

#### 2. The **git** version of `probe-run`:

<!-- TODO: update this once defmt is on crates.io? -->
``` console
$ cargo install \
    --git https://github.com/knurling-rs/probe-run \
    --branch main \
    --features defmt
```

#### 3. Useful tools:

```
$ cargo install cargo-generate cargo-bloat
```
## Usage

#### 1. Clone the code
#### 2. Attach the driver using a debug probe
#### 3. If needed, adjust slave address in code
#### 4. Flash the code

```bash
cargo run --release
```

## Acknowledgements

KM3-rs firmware was developed using tools that are part of the [Knurling] project, [Ferrous Systems]' effort at improving tooling used to develop for embedded systems. Various libraries contributed by the Rust and embedded Rust communities were also used.

Development of the firmware was sponsored by the Robotics and AI research group at Faculty of Electrical Engineering and Communications at Brno University of Technology.
## License
MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

[Knurling]: https://github.com/knurling-rs/meta
[Ferrous Systems]: https://ferrous-systems.com/
