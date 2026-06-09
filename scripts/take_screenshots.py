"""Capture screenshots of opti-route via Playwright for the README."""
import asyncio
from pathlib import Path
from playwright.async_api import async_playwright

URL = "http://localhost:8765/"
OUT = Path(__file__).resolve().parent.parent / "docs" / "img"
OUT.mkdir(parents=True, exist_ok=True)


async def main():
    async with async_playwright() as p:
        browser = await p.chromium.launch(headless=True)

        # --- desktop ---
        ctx = await browser.new_context(
            viewport={"width": 1440, "height": 900},
            device_scale_factor=2,
        )
        page = await ctx.new_page()
        await page.goto(URL)
        # let initial scene generate + render
        await page.wait_for_selector("svg#canvas circle", timeout=15000)
        await page.wait_for_timeout(800)

        await page.screenshot(path=OUT / "01-overview.png", full_page=False)

        # solve KSP
        await page.click("button.solver[data-algo='ksp']")
        await page.wait_for_function(
            "() => document.querySelector('svg#canvas polyline.edge.route')",
            timeout=30000,
        )
        await page.wait_for_timeout(500)
        await page.screenshot(path=OUT / "02-ksp-solved.png")

        # solve MIP
        await page.click("button.solver[data-algo='mip']")
        # wait until activeAlgo flips to mip — assignment count >0 of mip
        await page.wait_for_function(
            "() => document.querySelector('button.solver[data-algo=\"mip\"].active')",
            timeout=60000,
        )
        await page.wait_for_timeout(500)
        await page.screenshot(path=OUT / "03-mip-solved.png")

        # zoom into the sidebar (left 360px)
        await page.screenshot(
            path=OUT / "04-sidebar.png",
            clip={"x": 0, "y": 0, "width": 340, "height": 900},
        )

        # zoom into the comparison table — scroll panels to make sure it's visible
        await page.eval_on_selector(
            "details[open] summary:has(h2:has-text('結果比較'))",
            "el => el.scrollIntoView({block:'center'})",
        )
        await page.wait_for_timeout(300)
        await page.screenshot(
            path=OUT / "05-comparison.png",
            clip={"x": 0, "y": 320, "width": 340, "height": 360},
        )

        await ctx.close()

        # --- mobile ---
        ctx2 = await browser.new_context(
            viewport={"width": 390, "height": 844},  # iPhone 14
            device_scale_factor=3,
            user_agent=(
                "Mozilla/5.0 (iPhone; CPU iPhone OS 17_0 like Mac OS X) "
                "AppleWebKit/605.1.15 (KHTML, like Gecko) Version/17.0 "
                "Mobile/15E148 Safari/604.1"
            ),
        )
        page2 = await ctx2.new_page()
        await page2.goto(URL)
        await page2.wait_for_selector("svg#canvas circle", timeout=15000)
        await page2.wait_for_timeout(800)
        await page2.screenshot(path=OUT / "06-mobile.png", full_page=False)

        await ctx2.close()
        await browser.close()

    print("screenshots saved →", OUT)
    for f in sorted(OUT.glob("*.png")):
        print(" ", f.name, f.stat().st_size // 1024, "KB")


asyncio.run(main())
