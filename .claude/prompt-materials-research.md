# Prompt: research the robot PoC materials under $310 delivered

Paste the prompt below into Claude Code from the repository root.

---

You are researching and updating the physical-materials plan for this ROS 2 assistive-robot project. This is a research and documentation task, not authorization to order parts, change software, or test hardware.

## Goal and budget

Produce a technically compatible, complete shopping list for the room-mapping proof of concept (PoC) with a total purchase price of **less than $310 USD including shipping**. The PoC is a person-operated, remotely controlled car/base that carries a lidar and maps one room. It is not autonomous navigation. Clearly identify what is already owned and exclude only those items from the purchase total; do not silently omit parts needed to make the PoC work.

The eventual robot is a separate phase: it must travel between multiple rooms and carry about 20 lb of payload. Do not claim that a toy RC car, PoC chassis, motor, battery, or budget can meet that final requirement without verified load, traction, runtime, threshold, and safety data. Explain what PoC components could plausibly be reused and what must be selected or sized later. Do not force the final robot into the $310 PoC budget unless the user explicitly says that cap applies to both phases.

## Read the project first

1. Read `AGENTS.md` and follow its safety rules.
2. Read `docs/poc-plan.md`, `docs/hazard-analysis.md`, `docs/safety-test-procedure.md`, `docs/decisions/0001-ros-baseline.md`, and `docs/README.md`.
3. Inspect the relevant existing ROS packages and configuration only to understand hardware interfaces and constraints. Do not modify source code, configuration, derived safety distances, or hardware parameters.
4. Check the git working tree before editing. Preserve all pre-existing user changes.

The existing `docs/poc-plan.md` shopping list is a starting point, not verified truth. It currently describes a custom four-motor chassis, lidar, motor drivers, battery, Pico, and wiring, but gives approximate marketplace prices, excludes shipping, and contains a possible inconsistency about whether level shifters are required. Reconcile these against actual specifications and the user's clarified PoC: a remotely controlled car maps one room. Do not assume this means the currently listed mecanum chassis unless the project evidence supports that choice.

## ECC and delegation

First identify what “ECC” (Everything Claude Code) tooling is actually installed and available in this session. Read the applicable ECC instructions/skills and use their supported workflow; do not invent commands, agent names, or capabilities. If ECC is unavailable, continue with Claude Code's available tools and state that limitation.

If agent/delegate support is available, use separate, read-only agents for these independent checks:

- **PoC engineering:** identify the minimum complete, compatible hardware for remote driving plus room mapping; inspect power, mounting, control, odometry/localization assumptions, and safety gaps.
- **Sourcing:** find current purchase listings for each required item, exact variants, item price, availability, shipping to the specified destination, and seller/manufacturer specifications.
- **Independent audit:** verify each candidate URL resolves to the named product and exact variant, recheck electrical/mechanical compatibility, and independently recalculate all quantities and totals.

Give agents disjoint questions and require source URLs/evidence in their reports. The main session must reconcile conflicts and perform the final arithmetic itself. Do not delegate edits. If no suitable agents are available, do these checks sequentially and report that.

## Destination and shipping gate

Before asserting any delivered total, establish the shipping destination. Use a destination already explicitly provided by the user in this task/session. Otherwise ask the user for the country and postal/ZIP code, then pause before claiming a shipping-inclusive BOM. Do not use an arbitrary seller page location, guessed shipping fee, Prime assumption, free-shipping threshold, or an unrelated ZIP as the user's destination.

The required cap is the sum of the selected quantities' item prices plus shipping/handling and mandatory seller fees. State tax separately because the stated limit is “including shipping”; also show a tax-included estimate if the checkout price can be verified. If customs, import, or battery surcharges apply, include them in the cap. Do not round down. A total of exactly $310 does not pass: the total must be below $310.

## Evidence and price rules

- Browse the actual seller/manufacturer pages, not search snippets, ads, affiliate pages, or another model's page.
- Verify every link opens the exact named product and selected configuration/variant. Use direct, stable product URLs; remove tracking/redirect URLs when possible. Never invent or reconstruct a product URL from a guessed ID.
- For every line item, record the date checked, seller, exact item/variant, quantity, item price, shipping to the destination, line total, stock status, and a source URL. Distinguish observed facts from assumptions.
- Check the product specs against manufacturer documentation where available. For marketplace hardware, confirm variant details with the listing and flag details requiring seller confirmation. Do not treat a title, AI-generated summary, review, or photo as definitive electrical or load-rating evidence.
- Show each seller's subtotal and shipping separately, plus item subtotal, shipping total, mandatory fees, pre-tax delivered total, and tax separately. Account for minimum order values and shipping thresholds without counting unneeded products to reach them.
- If a seller will not reveal shipping without checkout/account/location, label it **unverified**. Do not represent the BOM as meeting the budget until all required shipping is evidenced. Give a conservative maximum only if the seller provides one.
- Provide a second source or viable substitute for costly/critical parts when possible, but only count one option in the selected total.
- Do not claim the lidar, chassis, motors, battery/BMS, charger, motor drivers, controller, or safety hardware are compatible unless the evidence supports their voltage/current/interfaces, dimensions, wheel-sensor output, load, and intended use.

## Engineering and safety constraints

- Follow the motion and software-safety constraints in `AGENTS.md`. The PoC must not bypass `robot_safety`, route around the software stop, or imply that a software stop replaces a physical emergency stop.
- Respect the project's stated fail-closed behavior for loss of command stream. Identify the need for independent motor power interruption, suitable fuse/protection, and a physical emergency stop as required for any powered floor test; include their cost if they are required for the proposed test, or explicitly classify the list as parts-only/not-floor-test-ready.
- Do not recommend using the project with an elder or other person in the operating area during an unvalidated test. Note tether, exclusion zone, second operator, and the complete safety test procedure requirements.
- Do not infer wheel odometry from a car that has no accessible encoder feedback. State what localization/mapping approach the PoC can actually support. A remote camera/driver view is not a substitute for lidar or pose estimation.
- Check that the mapping compute platform can run the project's ROS 2 Jazzy environment and chosen SLAM/lidar driver. The repo documents a Raspberry Pi 4, but its RAM is not yet known; call this out rather than assuming it is sufficient.
- Separate “required to demonstrate lidar mapping” from “required to drive the platform” and “required before floor testing.” Identify already-owned dependencies such as the Pi, laptop, power bank, SD card, cables, and tools from repo evidence; if ownership is uncertain, ask rather than assume.
- The final 20 lb payload robot needs a separate, evidence-based design: total mass includes payload and battery; show how rated wheel/motor torque, reduction, speed, floor transitions, stability, braking, battery/BMS current, and safety margins would be established. Do not size or certify this final robot from marketing payload numbers alone.

## Deliverable

Prepare a concise, evidence-backed replacement or revision for `docs/poc-plan.md`'s parts/shopping-list section. Preserve the existing staged plan and safety warnings unless evidence requires correcting them. Include:

1. A short conclusion stating whether a complete, working, shipping-verified PoC BOM is below $310. If not, say so plainly and show the smallest technically honest alternative or the unresolved blocker; never drop required parts to manufacture a passing total.
2. A BOM table with part/job, exact selected product and variant, quantity, owner status (buy/already owned/unknown), item price, shipping, delivered line total, seller, source link, and compatibility/spec evidence.
3. A transparent arithmetic breakdown by seller and a clear pre-tax delivered total. Keep the final robot's 20 lb payload materials and costs in a separate “future design, not included” section.
4. A concise list of unresolved specifications and questions, with who must verify each one and why it matters.
5. A short correction log for discrepancies found in the old list, especially omitted shipping, estimated prices, ambiguous RC-car-vs-custom-chassis scope, optional level shifters, and safety-critical exclusions.

Do not edit files until research, independent checks, and arithmetic are complete. Then make only the focused documentation edit to `docs/poc-plan.md` if evidence is sufficient. Do not commit. If location or any critical compatibility/shipping evidence is missing, ask the user or mark the BOM unverified instead of guessing. After editing, inspect the diff and run no hardware commands or tests that could move a robot.

---